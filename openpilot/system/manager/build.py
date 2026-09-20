#!/usr/bin/env python3
import hashlib
import os
from pathlib import Path
import shutil
import subprocess

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.hardware import HARDWARE, AGNOS


def sync_python_env(report=print) -> None:
  """Reconcile the project venv after an update changes the pinned lockfile."""
  lock = Path(BASEDIR) / "uv.lock"
  if not lock.exists():
    return
  digest = hashlib.sha256(lock.read_bytes()).hexdigest()
  # Match uv's project environment selection, including isolated build/test envs.
  environment = Path(os.environ.get("UV_PROJECT_ENVIRONMENT", ".venv"))
  if not environment.is_absolute():
    environment = Path(BASEDIR) / environment
  marker = environment / ".op_synced_lock"
  if marker.exists() and marker.read_text().strip() == digest:
    return

  uv = shutil.which("uv") or str(Path.home() / ".local/bin/uv")
  # Keep the lockfile frozen and preserve unrelated installed extras. Never mark
  # a missing uv, interrupted install, or failed install as successfully synced.
  command = [uv, "sync", "--frozen", "--inexact"]
  with subprocess.Popen(command, cwd=BASEDIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
    assert process.stdout is not None
    for line in process.stdout:
      report(line.rstrip())
    if process.wait() != 0:
      raise subprocess.CalledProcessError(process.returncode, command)

  marker.parent.mkdir(parents=True, exist_ok=True)
  temporary = marker.with_suffix(".tmp")
  temporary.write_text(digest)
  temporary.replace(marker)


def build() -> None:
  spinner = Spinner()
  spinner.update_progress(0, 100)

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  try:
    sync_python_env()
  except (subprocess.CalledProcessError, OSError) as error:
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow(f"Failed to update dependencies: {error}\nCheck the internet connection, then reboot.") as window:
        window.wait_for_exit()
    raise SystemExit(1) from error

  # building with all cores can result in using too much memory, so retry serially
  compile_output: list[bytes] = []
  for parallelism in ([], ["-j4"], ["-j1"]):
    compile_output.clear()
    with subprocess.Popen(["scons", *parallelism], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR}, stderr=subprocess.PIPE) as scons:
      assert scons.stderr is not None

      # Read progress from stderr and update spinner
      while scons.poll() is None:
        try:
          line = scons.stderr.readline()
          if line is None:
            continue
          line = line.rstrip()

          prefix = b'progress: '
          if line.startswith(prefix):
            progress = float(line[len(prefix):])
            spinner.update_progress(100 * min(1., progress / 100.), 100.)
          elif len(line):
            compile_output.append(line)
            print(line.decode('utf8', 'replace'))
        except Exception:
          pass

      # Drain and close the pipe before retrying or returning.
      for line in scons.stderr.read().split(b'\n'):
        line = line.rstrip()
        if len(line):
          compile_output.append(line)

    if scons.returncode == 0:
      break

  os.sync()

  if scons.returncode != 0:
    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

if __name__ == "__main__":
  build()
