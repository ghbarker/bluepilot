"""Render the actual MICI graph, fonts and diagnostics under Linux/Xvfb."""
import os
from pathlib import Path
import subprocess
import sys


def test_mici_lateral_diagnostics_render(tmp_path):
  result = subprocess.run(['xvfb-run', '-a', sys.executable, str(Path(__file__).resolve()), str(tmp_path / 'params')],
                          env={**os.environ, 'BIG': '0', 'SCALE': '1'}, capture_output=True, text=True, timeout=40)
  assert result.returncode == 0, result.stdout + result.stderr


def render_check(params_dir):
  import time
  from unittest.mock import patch
  from openpilot.common.params import Params

  params_init = Params.__init__

  def isolated_params(self, d=''):
    return params_init(self, d or params_dir)

  with patch.object(Params, '__init__', isolated_params):
    import pyray as rl
    from openpilot.cereal import messaging
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.bp.mici.onroad.lateral_debug_mici import LateralDebugMici
    from openpilot.system.ui.lib.application import gui_app

    sm = ui_state.sm
    delay = messaging.new_message('lateralDelay', valid=True)
    delay.lateralDelay.lateralDelay = .372
    delay.lateralDelay.status = 'estimated'
    controller = messaging.new_message('controllerStateBP', valid=True)
    controller.controllerStateBP.bmsAngleAutoCalibrate = True
    gui_app.init_window('Lateral diagnostics regression')
    try:
      with patch.object(ui_state, 'started', True):
        widget = LateralDebugMici(lambda: None)
        rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
        for status in ('locked', '{"pause":"delay"}',
                       '{"low":{"ph":"collect","w":5,"need":10},"high":{"ph":"collect","w":4,"need":10}}',
                       '{"low":{"ph":"collect","reason":[]},"high":{"ph":"collect"}}'):
          controller.controllerStateBP.bmsAngleAutoCalState = status
          now_ns = time.monotonic_ns()
          delay.logMonoTime = now_ns
          controller.logMonoTime = now_ns
          sm.update_msgs(time.monotonic(), [delay, controller])
          sm.valid['lateralDelay'] = sm.alive['lateralDelay'] = True
          sm.valid['controllerStateBP'] = sm.alive['controllerStateBP'] = True
          widget._next_diagnostic_time = 0
          widget._update_state()
          assert '0.372s live' in widget._graph._config.title
          rl.begin_drawing()
          try:
            rl.clear_background(rl.BLACK)
            widget.render(rect)
          finally:
            rl.end_drawing()
        sm.alive['controllerStateBP'] = False
        sm.alive['lateralDelay'] = False
        widget._next_diagnostic_time = 0
        widget._update_state()
        assert widget._diagnostic_text == 'Auto-cal: unavailable'
        assert 'live' not in widget._graph._config.title
    finally:
      gui_app.close()


if __name__ == '__main__':
  render_check(sys.argv[1])
