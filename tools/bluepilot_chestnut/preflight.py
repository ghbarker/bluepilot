#!/usr/bin/env python3
"""Verify the pinned driving model and preserved Chestnut base before building."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
STAGING = "dd29072b71a06c9ce125a481ced431ba995b84f6"
SOURCE = "a5f44653d7f43ad57fef2f546f3916ec4cbf3c56"
DONOR = "e22afa6be9b881fa784c92ebb316db47728a3d81"

# The donor adds Ford control and presentation. These new-base systems must stay
# intact; native build definitions are restored separately from pinned source.
PROTECTED = [
  "openpilot/sunnypilot/models",
  "openpilot/selfdrive/modeld/modeld.py",
  "openpilot/selfdrive/modeld/helpers.py",
  "openpilot/selfdrive/monitoring",
  "openpilot/selfdrive/controls/lib/longcontrol.py",
  "openpilot/common/hardware/comma/agnos.json",
  "opendbc_repo/opendbc/safety/lateral.h",
]


def sha256(path):
  with path.open("rb") as stream:
    return hashlib.file_digest(stream, "sha256").hexdigest()


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



def check(models_only=False):
  manifest = json.loads((HERE / "model_artifacts.json").read_text())
  verify_model_files(ROOT, manifest)
  if not models_only:
    subprocess.run(["git", "merge-base", "--is-ancestor", STAGING, "HEAD"], cwd=ROOT, check=True)
    subprocess.run(["git", "diff", "--quiet", STAGING, "--", *PROTECTED], cwd=ROOT, check=True)
    if (ROOT / "prebuilt").exists():
      raise ValueError("Modified native code requires a rebuild; remove the prebuilt marker")
    if "export SKIP_TINYGRAD_COMPILE=1" not in (ROOT / "launch_env.sh").read_text():
      raise ValueError("Driving-model compilation must stay disabled for this overlay")
    for asset in json.loads((HERE / "restored_assets.json").read_text()):
      path = ROOT / asset["path"]
      if path.stat().st_size != asset["size"] or sha256(path) != asset["oid"]:
        raise ValueError(f"Donor asset changed: {asset['path']}")
  return {
    "status": "MODEL_INTEGRITY_VERIFIED" if models_only else "SOURCE_INTEGRITY_VERIFIED",
    "staging": STAGING, "source": SOURCE, "donor": DONOR,
    "chestnut_chunks_sha256_verified": 18,
    "device_qualification": "not performed",
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--models-only", action="store_true", help="Check artifacts before a device build, without Git history")
  args = parser.parse_args()
  try:
    print(json.dumps(check(args.models_only), indent=2))
    return 0
  except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
    print(f"Preflight failed: {error}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
