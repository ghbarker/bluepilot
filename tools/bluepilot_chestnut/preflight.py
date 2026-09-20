#!/usr/bin/env python3
"""Read-only readiness checks for the pinned BluePilot/Chestnut port.

This is a development preflight, not an installer or a safety certification.
Exit 2 means the overlay is blocked; exit 1 means a preflight check failed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
STAGING = "dd29072b71a06c9ce125a481ced431ba995b84f6"
SOURCE = "a5f44653d7f43ad57fef2f546f3916ec4cbf3c56"
DONOR = "e22afa6be9b881fa784c92ebb316db47728a3d81"
OLD_OPENDBC = "b9712d20efd4fb7b1c29378ee153013d2a32a9f1"
NEW_OPENDBC = "f95f996f5917dcbbf2e32fe51b606a24cf836af6"
RUNTIME_PATHS = ["openpilot", "opendbc_repo", "panda", "msgq_repo", "tinygrad_repo",
                 "rednose_repo", "teleoprtc_repo", "system", "scripts", "launch_chffrplus.sh",
                 "launch_env.sh", "launch_openpilot.sh", "prebuilt", "pyproject.toml", "uv.lock"]


def git(*args):
  return subprocess.check_output(["git", *args], cwd=ROOT)


def sha256(path):
  with path.open("rb") as stream:
    return hashlib.file_digest(stream, "sha256").hexdigest()


def angle_fields(header):
  match = re.search(r"typedef struct \{([^{}]*)\} AngleSteeringLimits;", header, re.S)
  if match is None:
    raise ValueError("Cannot identify AngleSteeringLimits; reassess the safety API")
  return set(re.findall(r"\b(\w+)\s*;", match.group(1)))


def missing_legacy_fields(header):
  legacy = {"max_angle_error", "angle_error_min_speed", "angle_is_curvature",
            "enforce_angle_error", "inactive_angle_is_zero"}
  return sorted(legacy - angle_fields(header))


def verify_model_files(root, manifest):
  expected = manifest["files"]
  model_dir = root / manifest["directory"]
  actual = {p.name for p in model_dir.glob("big_driving_tinygrad.pkl.*")}
  if actual != set(expected):
    raise ValueError(f"Model file set changed: missing={set(expected) - actual}, extra={actual - set(expected)}")
  for name, metadata in expected.items():
    path = model_dir / name
    if path.stat().st_size != metadata["bytes"] or sha256(path) != metadata["sha256"]:
      raise ValueError(f"Chestnut artifact changed: {name}")
  count = int((model_dir / "big_driving_tinygrad.pkl.chunkmanifest").read_text())
  chunks = {f"big_driving_tinygrad.pkl.chunk{i:02d}of{count:02d}" for i in range(1, count + 1)}
  if count != 18 or chunks != actual - {"big_driving_tinygrad.pkl.chunkmanifest"}:
    raise ValueError("Unexpected Chestnut chunk count or names")


def check():
  git("merge-base", "--is-ancestor", STAGING, "HEAD")
  # Working-tree diff includes staged and unstaged modifications to tracked files.
  # This preparation commit must leave the runnable release exactly unchanged.
  changed = git("diff", "--name-only", STAGING, "--", *RUNTIME_PATHS).decode().splitlines()
  if changed:
    raise ValueError(f"Runtime changed before the safety port was validated: {changed}")
  untracked = git("ls-files", "--others", "--exclude-standard", "--", *RUNTIME_PATHS).decode().splitlines()
  if untracked:
    raise ValueError(f"Untracked runtime files require review: {untracked}")
  manifest = json.loads((HERE / "model_artifacts.json").read_text())
  verify_model_files(ROOT, manifest)
  header = (ROOT / "opendbc_repo/opendbc/safety/declarations.h").read_text()
  missing = missing_legacy_fields(header)
  if not missing:
    raise ValueError("Expected API discrepancy disappeared; reassess rather than assume readiness")
  return {
    "status": "BLOCKED_NOT_INSTALL_READY",
    "staging": STAGING,
    "source": SOURCE,
    "donor": DONOR,
    "runtime_unchanged": True,
    "chestnut_chunks_sha256_verified": 18,
    "donor_AngleSteeringLimits_fields_absent_upstream": missing,
    "remaining": [
      "Port and validate Ford angle/shadow/pinion checks against current curvature safety state without weakening current safeguards",
      "Resolve overlay conflicts and current Python/UI/Params/schema interfaces",
      "Reconstruct the exact source and dependency build inputs; rebuild native code and Panda firmware on suitable hardware",
      "Verify the rebuilt package and model selection/load on comma four plus Chestnut",
    ],
  }


def compile_donor_probe():
  """Compile a donor header against current shared safety, in temporary files only.

  A diagnostic experiment, NOT the intended merge: no old shared safety files are
  installed. The donor commit must already have been fetched explicitly.
  """
  compiler = shutil.which("cc")
  if compiler is None:
    raise RuntimeError("A Linux C compiler is required for --probe-donor")
  donor = git("show", f"{DONOR}:opendbc_repo/opendbc/safety/modes/ford.h")
  with tempfile.TemporaryDirectory(prefix="bluepilot-safety-probe-") as scratch:
    include = Path(scratch) / "opendbc/safety/modes"
    include.mkdir(parents=True)
    (include / "ford.h").write_bytes(donor)
    source = ROOT / "opendbc_repo/opendbc/safety/tests/libsafety/safety.c"
    result = subprocess.run([
      compiler, "-fsyntax-only", "-std=gnu11", "-DALLOW_DEBUG", "-Wfatal-errors",
      "-I", scratch, "-I", str(ROOT / "opendbc_repo"), str(source),
    ], cwd=ROOT, capture_output=True, text=True)
    diagnostic = result.stdout + result.stderr
    print(diagnostic)
    if result.returncode == 0:
      raise RuntimeError("Donor unexpectedly compiled; investigate before changing the readiness status")
    if "AngleSteeringLimits" not in diagnostic or "max_angle_error" not in diagnostic:
      raise RuntimeError("Probe failed for a different reason; this is not evidence of the expected API conflict")
    print("Confirmed: donor header is incompatible with the current shared safety API. No runtime files changed.")


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--probe-donor", action="store_true", help="Run the negative C compatibility probe (Linux)")
  args = parser.parse_args()
  try:
    report = check()
    print(json.dumps(report, indent=2))
    if args.probe_donor:
      compile_donor_probe()
    return 2
  except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
    print(f"Preflight failed: {error}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
