#!/usr/bin/env python3
"""Reproduce the screened text-merge trial without changing runtime files.

All pinned commits must be fetched beforehand. The report includes merge status,
not resolved code. No candidate is installed or treated as API-compatible.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import tempfile

try:
  from .preflight import DONOR, OLD_OPENDBC, ROOT, STAGING, git
except ImportError:
  from preflight import DONOR, OLD_OPENDBC, ROOT, STAGING, git

COMMON_ANCESTOR = "01a843e0acbe74d566a7eee9fe0f12f227ae81ed"
CODE_ROOTS = ("bluepilot/", "common/", "cereal/", "selfdrive/", "sunnypilot/", "system/", "third_party/")
SHARED_OPENDBC = {"opendbc/car/car_helpers.py", "opendbc/car/structs.py", "opendbc/sunnypilot/car/interfaces.py"}


def tree(ref):
  files = {}
  for record in git("ls-tree", "-rz", ref).split(b"\0"):
    if record:
      metadata, name = record.split(b"\t", 1)
      mode, kind, oid = metadata.decode().split()
      files[name.decode()] = (mode, kind, oid)
  return files


def blob(entry):
  if entry[1] != "blob":
    raise ValueError(f"Expected a blob, got {entry[1]}")
  return git("cat-file", "blob", entry[2])


def mapped(path):
  if path.startswith(CODE_ROOTS[1:]):
    path = "openpilot/" + path
  return path.replace("openpilot/system/hardware/tici/", "openpilot/common/hardware/comma/")


def collect():
  current = tree(STAGING)
  rows = []
  with tempfile.TemporaryDirectory(prefix="bluepilot-merge-trial-") as scratch:
    scratch = Path(scratch)
    for base, donor, prefix in [(COMMON_ANCESTOR, DONOR, ""), (OLD_OPENDBC, f"{DONOR}:opendbc_repo", "opendbc_repo/")]:
      before, after = tree(base), tree(donor)
      changes = git("diff", "--no-renames", "--name-status", base, donor).decode().splitlines()
      for change in changes:
        status, path = change.split("\t", 1)
        if status not in {"A", "M"}:
          continue
        if not prefix and not path.startswith(CODE_ROOTS) and path not in {"BPVERSION", "BP_CHANGES.json"}:
          continue
        if prefix and not path.startswith(("opendbc/car/ford/", "opendbc/sunnypilot/car/ford/", "opendbc/safety/")) and path not in SHARED_OPENDBC:
          continue
        destination = prefix + path if prefix else mapped(path)
        row = {"source": prefix + path, "destination": destination, "status": status}
        old = blob(before[path]) if status == "M" else None
        incoming = blob(after[path])
        existing = blob(current[destination]) if destination in current else None
        if existing is None:
          result = "missing-current" if old is not None else "new-at-mapped-path"
        elif existing == incoming:
          result = "identical"
        elif b"\0" in existing or b"\0" in incoming:
          result = "binary-review"
        else:
          paths = [scratch / name for name in ("current", "base", "donor")]
          for target, data in zip(paths, (existing, old or b"", incoming), strict=True):
            target.write_bytes(data)
          merge = subprocess.run(["git", "merge-file", "-p", *map(str, paths)], cwd=ROOT, capture_output=True)
          if merge.returncode < 0 or merge.returncode > 127:
            raise RuntimeError(merge.stderr.decode())
          result = "text-merge-only" if merge.returncode == 0 else "text-conflict"
        row["result"] = result
        rows.append(row)
  return rows


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--compare", action="store_true", help="Compare against committed candidate inventory")
  args = parser.parse_args()
  rows = collect()
  if args.compare:
    expected = json.loads((Path(__file__).parent / "candidate_inventory.json").read_text())["files"]
    if rows != expected:
      raise SystemExit("Candidate inventory differs; inspect the changed entries")
    print("Reproduced the committed screened inventory")
  else:
    print(json.dumps(rows, indent=2))
  print(dict(Counter(row["result"] for row in rows)))


if __name__ == "__main__":
  main()
