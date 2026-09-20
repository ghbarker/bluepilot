#!/usr/bin/env python3
# BluePilot: verbose build with the current build retries and dependency sync. Drives
# the BP spinner (common/bp_spinner.py -> system/ui/bp_spinner.py) so an on-device build shows a
# progress bar + the live scons line, and a scrollable build log on failure instead of a black
# screen. Wired in via launch_chffrplus.sh (./bp_build.py in place of ./build.py).
import os
import subprocess
from typing import NoReturn

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.bp_spinner import BPSpinner
from openpilot.common.hardware import HARDWARE, AGNOS
from openpilot.system.manager.build import sync_python_env

def _fail(spinner: BPSpinner, message: str) -> NoReturn:
  for line in message.split("\n"):
    if line:
      spinner.update(f"0|{line}")
  spinner.build_failed()
  if not os.getenv("CI"):
    spinner.wait_for_exit()  # stays on the error screen until the user reboots
  spinner.close()
  exit(1)


def build() -> None:
  spinner = BPSpinner()
  spinner.update_progress_with_text(0, 100, "Starting build...")

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # reconcile the venv with the checked-out lockfile before building
  try:
    spinner.update_progress_with_text(0, 100, "Checking Python dependencies...")
    sync_python_env(lambda line: spinner.update(f"0|{line}"))
  except (subprocess.CalledProcessError, OSError) as error:
    _fail(spinner, f"Failed to update dependencies: {error}\nEnsure the device has an internet connection, then reboot.")

  last_status = "Building..."
  compile_output: list[bytes] = []
  for attempt, parallelism in enumerate(([], ["-j4"], ["-j1"])):
    compile_output.clear()
    progress = 0.0
    if attempt > 0:
      spinner.build_retry()
      last_status = f"Retrying build ({' '.join(parallelism) or 'all cores'})..."
      spinner.update_progress_with_text(0, 100, last_status)

    try:
      process = subprocess.Popen(["scons", *parallelism], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR},
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except OSError as error:
      _fail(spinner, f"Could not start the build: {error}")
    with process as scons:
      assert scons.stdout is not None

      while scons.poll() is None:
        try:
          line = scons.stdout.readline()
          if not line:
            continue
          line = line.rstrip()

          prefix = b'progress: '
          if line.startswith(prefix):
            progress = min(100.0, float(line[len(prefix):]))
            spinner.update_progress_with_text(progress, 100, last_status)
          elif len(line):
            compile_output.append(line)
            last_status = line.decode('utf8', 'replace')
            print(last_status)
            spinner.update(f"{round(progress)}|{last_status}")
        except Exception:
          pass

      # drain remaining output before retrying or returning
      for line in scons.stdout.read().split(b'\n'):
        line = line.rstrip()
        if len(line):
          compile_output.append(line)
          try:
            spinner.update(f"{round(progress)}|{line.decode('utf8', 'replace')}")
          except Exception:
            pass

    if scons.returncode == 0:
      break

  os.sync()

  if scons.returncode != 0:
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')
    print(error_s)
    _fail(spinner, "")  # build output already streamed into the spinner's log buffer

  spinner.update_progress_with_text(100, 100, "Build complete")
  spinner.close()


if __name__ == "__main__":
  build()
