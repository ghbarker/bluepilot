import hashlib
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import Mock

import pytest

from openpilot.system.manager import build, bp_build


@pytest.fixture
def sync_project(tmp_path, monkeypatch):
  monkeypatch.setattr(build, 'BASEDIR', str(tmp_path))
  monkeypatch.setenv('UV_PROJECT_ENVIRONMENT', '.venv')
  (tmp_path / 'uv.lock').write_text('first lock')
  uv = tmp_path / 'uv'
  uv.write_text(f'''#!{sys.executable}
import pathlib, sys
assert sys.argv[1:] == ['sync', '--frozen', '--inexact']
with open('attempts', 'a') as f:
  f.write('sync\\n')
print('updating pinned dependencies', flush=True)
sys.exit(1 if pathlib.Path('fail').exists() else 0)
''')
  uv.chmod(0o755)
  monkeypatch.setattr(build.shutil, 'which', lambda _: str(uv))
  return tmp_path


def test_dependency_refresh_only_after_lock_change(sync_project):
  root = sync_project
  output = []
  build.sync_python_env(output.append)
  marker = root / '.venv/.op_synced_lock'
  assert marker.read_text() == hashlib.sha256((root / 'uv.lock').read_bytes()).hexdigest()
  assert output == ['updating pinned dependencies']
  build.sync_python_env(output.append)
  assert (root / 'attempts').read_text().splitlines() == ['sync']
  (root / 'uv.lock').write_text('updated lock')
  build.sync_python_env(output.append)
  assert len((root / 'attempts').read_text().splitlines()) == 2
  marker.unlink()  # a recreated environment needs synchronization again
  build.sync_python_env()
  assert len((root / 'attempts').read_text().splitlines()) == 3


def test_failed_dependency_refresh_is_retried_and_never_marked_synced(sync_project):
  root = sync_project
  build.sync_python_env()
  marker = root / '.venv/.op_synced_lock'
  successful = marker.read_text()
  (root / 'uv.lock').write_text('new dependencies')
  (root / 'fail').touch()
  for _ in range(2):
    with pytest.raises(subprocess.CalledProcessError):
      build.sync_python_env()
    assert marker.read_text() == successful
  (root / 'fail').unlink()
  build.sync_python_env()
  assert marker.read_text() != successful


def test_marker_follows_explicit_project_environment(sync_project, monkeypatch):
  environment = sync_project / 'isolated-build-env'
  monkeypatch.setenv('UV_PROJECT_ENVIRONMENT', str(environment))
  build.sync_python_env()
  assert (environment / '.op_synced_lock').exists()
  assert not (sync_project / '.venv/.op_synced_lock').exists()


def test_missing_uv_does_not_bypass_failed_dependency_setup(sync_project, monkeypatch):
  monkeypatch.setattr(build.shutil, 'which', lambda _: str(sync_project / 'missing-uv'))
  with pytest.raises(FileNotFoundError):
    build.sync_python_env()
  assert not (sync_project / '.venv/.op_synced_lock').exists()


@pytest.mark.parametrize('success_attempt', [3, 4])
def test_verbose_build_retries_and_preserves_failure_output(tmp_path, monkeypatch, success_attempt):
  scons = tmp_path / 'scons'
  scons.write_text(f'''#!{sys.executable}
import pathlib, sys
p = pathlib.Path('attempts')
n = int(p.read_text()) + 1 if p.exists() else 1
p.write_text(str(n))
print('progress: 25', file=sys.stderr, flush=True)
print('compiling source', flush=True)
print('build error detail' if n < {success_attempt} else 'finished', file=sys.stderr)
sys.exit(1 if n < {success_attempt} else 0)
''')
  scons.chmod(0o755)
  monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
  monkeypatch.setenv('CI', '1')
  monkeypatch.setattr(bp_build, 'BASEDIR', str(tmp_path))
  monkeypatch.setattr(bp_build, 'AGNOS', False)
  monkeypatch.setattr(bp_build, 'HARDWARE', Mock())
  monkeypatch.setattr(bp_build, 'sync_python_env', Mock())
  monkeypatch.setattr(bp_build.os, 'sync', Mock())
  spinner = Mock()
  monkeypatch.setattr(bp_build, 'BPSpinner', lambda: spinner)
  if success_attempt == 4:
    with pytest.raises(SystemExit) as error:
      bp_build.build()
    assert error.value.code == 1
    spinner.build_failed.assert_called_once()
    assert any('build error detail' in call.args[0] for call in spinner.update.call_args_list)
  else:
    bp_build.build()
    spinner.build_failed.assert_not_called()
  assert (tmp_path / 'attempts').read_text() == '3'
  assert spinner.build_retry.call_count == 2
  spinner.close.assert_called_once()


def test_dependency_failure_prevents_compilation(monkeypatch):
  monkeypatch.setenv('CI', '1')
  monkeypatch.setattr(bp_build, 'AGNOS', False)
  monkeypatch.setattr(bp_build, 'HARDWARE', Mock())
  monkeypatch.setattr(bp_build, 'sync_python_env', Mock(side_effect=OSError('offline')))
  compiler = Mock()
  monkeypatch.setattr(bp_build.subprocess, 'Popen', compiler)
  spinner = Mock()
  monkeypatch.setattr(bp_build, 'BPSpinner', lambda: spinner)
  with pytest.raises(SystemExit):
    bp_build.build()
  compiler.assert_not_called()
  spinner.build_failed.assert_called_once()


def test_spinner_resolves_current_layout_and_tolerates_closed_pipe(monkeypatch):
  from openpilot.common import bp_spinner
  process = Mock()
  process.stdin.write.side_effect = BrokenPipeError
  factory = Mock(return_value=process)
  monkeypatch.setattr(bp_spinner.subprocess, 'Popen', factory)
  spinner = bp_spinner.BPSpinner()
  assert Path(factory.call_args.kwargs['cwd'], 'bp_spinner.py').is_file()
  spinner.update('starting')
  spinner.close()
