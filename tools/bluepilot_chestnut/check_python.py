#!/usr/bin/env python3
"""Catch syntax and unresolved-name mistakes in the Python overlay."""
import ast
import subprocess
import sys

from tools.bluepilot_chestnut.preflight import ROOT, STAGING


def main():
  changed = subprocess.check_output(
    ["git", "diff", "--name-only", "--diff-filter=ACMR", STAGING, "--", "*.py"], cwd=ROOT, text=True).splitlines()
  added = subprocess.check_output(
    ["git", "ls-files", "--others", "--exclude-standard", "--", "*.py"], cwd=ROOT, text=True).splitlines()
  files = sorted(set(changed + added))
  for name in files:
    ast.parse((ROOT / name).read_text(), filename=name)
  if files:
    subprocess.run([sys.executable, "-m", "ruff", "check", "--select", "F821,F822,F811", *files], cwd=ROOT, check=True)
  print(f"Parsed and checked {len(files)} changed Python files")


if __name__ == "__main__":
  main()
