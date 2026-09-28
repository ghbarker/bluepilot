"""Exercise the real comma-four update button with isolated params and no updater signals."""
import os
from pathlib import Path
import subprocess
import sys


def test_updater_disabled_missing_and_normal_responses(tmp_path):
  result = subprocess.run(['xvfb-run', '-a', sys.executable, str(Path(__file__).resolve()),
                           str(tmp_path / 'params')], env={**os.environ, 'BIG': '0', 'SCALE': '1'},
                          capture_output=True, text=True, timeout=40)
  assert result.returncode == 0, result.stdout + result.stderr


def exercise_button(params_dir):
  from unittest.mock import patch
  from openpilot.common.params import Params
  params_init = Params.__init__

  def isolated_params(self, d=''):
    return params_init(self, d or params_dir)

  with patch.object(Params, '__init__', isolated_params):
    import pyray as rl
    from openpilot.system.ui.lib.application import gui_app
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.mici.layouts.settings.software import CheckUpdateButton, UpdaterState

    gui_app.init_window('Updater status regression')
    try:
      params = Params()
      params.put_bool('DisableUpdates', True, block=True)
      with patch.object(ui_state, 'started', False), patch.object(subprocess, 'run') as signal:
        button = CheckUpdateButton()
        button._state = UpdaterState.UPDATER_RESPONDING
        button.set_rotate_icon(True)
        button._update_state()
        assert button.get_value() == 'updates disabled'
        assert button._state == UpdaterState.IDLE
        button.check_for_update()
        signal.assert_not_called()
        # Render the actual disabled state; no restart or updater writes requested.
        rl.begin_drawing()
        try:
          button.render(rl.Rectangle(0, 0, 402, 180))
        finally:
          rl.end_drawing()
        assert params.get_bool('DisableUpdates')
        assert not params.get_bool('DoReboot')
        assert params.get('UpdaterState') is None

        params.put_bool('DisableUpdates', False, block=True)
        button._update_state()
        assert button.get_value() == ''
        with patch.object(rl, 'get_time', return_value=100.):
          button.check_for_update()
          button._update_state()
          assert button._state == UpdaterState.WAITING_FOR_UPDATER
        with patch.object(rl, 'get_time', return_value=111.):
          button._update_state()
          assert button.get_value() == 'updater failed\nto respond'
          assert button._state == UpdaterState.IDLE

        # Retrying starts a fresh timeout; normal checking -> idle completes.
        with patch.object(rl, 'get_time', return_value=200.):
          button.check_for_update()
          params.put('UpdaterState', 'checking...', block=True)
          button._update_state()
          assert button._state == UpdaterState.UPDATER_RESPONDING
          button._update_state()
          assert button.get_value() == 'checking...'
          params.put('UpdaterState', 'idle', block=True)
          button._update_state()
          assert button._state == UpdaterState.IDLE
          button._update_state()
          assert button.get_value() == 'up to date'

        # A worker that disappears after responding must time out too.
        params.put('UpdaterState', 'checking...', block=True)
        with patch.object(rl, 'get_time', return_value=300.):
          button.check_for_update()
          button._update_state()
          params.remove('UpdaterState')
          button._update_state()
          assert button._state == UpdaterState.WAITING_FOR_UPDATER
        with patch.object(rl, 'get_time', return_value=311.):
          button._update_state()
          assert button.get_value() == 'updater failed\nto respond'
          assert button._state == UpdaterState.IDLE
    finally:
      gui_app.close()


if __name__ == '__main__':
  exercise_button(sys.argv[1])
