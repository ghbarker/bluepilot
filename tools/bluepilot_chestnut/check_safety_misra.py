#!/usr/bin/env python3
"""Run the pinned safety analyzer and verify it rejects deliberate Ford defects.

Uses the flags and suppression list from opendbc's test_misra.sh without running
its dependency installer or writing analyzer artifacts into the source checkout.
"""
import concurrent.futures
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import cppcheck


ROOT = Path(__file__).resolve().parents[2]
SAFETY = ROOT / "opendbc_repo/opendbc/safety"
CHECKER = Path(cppcheck.DIR)

# Keep the untouched control case: a broken analyzer must not make mutations pass.
MUTATIONS = (
  ("control_release", None, None),
  ("control_debug", None, None),
  ("composite_cast", "misra-c2012-10.8", (
    "      const uint16_t shadow_raw = (msg->data[5] << 8) | msg->data[6];\n" +
    "      ford_overlay_bp_shadow_curvature_raw = (int16_t)shadow_raw;",
    "      ford_overlay_bp_shadow_curvature_raw = (int16_t)((msg->data[5] << 8) | msg->data[6]);",
  )),
  ("early_return", "misra-c2012-15.5", (
    "  bool tx = ford_overlay_tx_hook(msg);",
    "  if (msg->addr == 0U) { return false; }\n  bool tx = ford_overlay_tx_hook(msg);",
  )),
)


def run_variant(folder: Path, variant: tuple, cc_include: str) -> tuple[str, bool, str]:
  name, rule, mutation = variant
  source_root = folder / name
  safety = source_root / "opendbc/safety"
  # Only C sources and the checked-in suppression file are analyzer inputs.
  for source in SAFETY.rglob("*"):
    if source.is_file() and (source.suffix in (".c", ".h") or source.name == "suppressions.txt"):
      target = safety / source.relative_to(SAFETY)
      target.parent.mkdir(parents=True, exist_ok=True)
      shutil.copyfile(source, target)
  if mutation is not None:
    target = safety / "modes" / ("ford_bluepilot.h" if name == "composite_cast" else "ford.h")
    before, after = mutation
    content = target.read_text()
    if content.count(before) != 1:
      return name, False, "Mutation target changed; review and update the analyzer regression."
    target.write_text(content.replace(before, after))

  command = [str(CHECKER / "cppcheck"), "--inline-suppr", "-I", str(source_root), "-I", cc_include,
             "--suppress=missingIncludeSystem", "--suppress=*:*include/*",
             "--suppressions-list=" + str(safety / "tests/misra/suppressions.txt"),
             "--error-exitcode=2", "--check-level=exhaustive", "--safety", "--platform=arm32-wchar_t4",
             "-DALLOW_DEBUG" if name == "control_debug" else "-UALLOW_DEBUG",
             "-D__GNUC__=9", "-D__has_include_next(x)=0", "--std=c11",
             "--enable=all", "--enable=unusedFunction", "--addon=misra", str(safety / "tests/misra/main.c")]
  result = subprocess.run(command, cwd=source_root, capture_output=True, text=True, check=False)
  output = result.stdout + result.stderr
  # Some MISRA failures return zero. Check diagnostic text as upstream does.
  findings = [line for line in output.splitlines() if re.search(r": (?:error|warning|style): ", line)]
  failed = result.returncode != 0 or bool(findings) or "misra violation" in output
  passed = not failed if rule is None else failed and f"[{rule}]" in output
  details = "\n".join(findings) if findings else output
  return name, passed, details


def main() -> int:
  coverage = subprocess.check_output(
    [sys.executable, str(CHECKER / "addons/misra.py"), "-generate-table"], text=True)
  if coverage != (SAFETY / "tests/misra/coverage_table").read_text():
    print("FAIL: pinned MISRA coverage table changed; review the analyzer dependency.")
    return 1
  cc_include = subprocess.check_output(["cc", "-print-file-name=include"], text=True).strip()
  with tempfile.TemporaryDirectory(prefix="bluepilot-misra-") as temp:
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(MUTATIONS)) as pool:
      results = list(pool.map(lambda variant: run_variant(Path(temp), variant, cc_include), MUTATIONS))
  for name, passed, details in results:
    print(f"{'PASS' if passed else 'FAIL'}: safety MISRA {name}")
    if not passed:
      print(details)
  return 0 if all(passed for _, passed, _ in results) else 1


if __name__ == "__main__":
  raise SystemExit(main())
