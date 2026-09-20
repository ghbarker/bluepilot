import os
from pathlib import Path
import subprocess
import sys


def test_steering_arc_direction_and_data_loss(tmp_path):
  result = subprocess.run(['xvfb-run', '-a', sys.executable, str(Path(__file__).resolve()), str(tmp_path / 'params')],
                          env={**os.environ, 'BIG': '0', 'SCALE': '1'}, capture_output=True, text=True, timeout=40)
  assert result.returncode == 0, result.stdout + result.stderr


def render_check(params_dir):
  import math
  import time
  from unittest.mock import patch
  from openpilot.common.params import Params

  params_init = Params.__init__
  def isolated_params(self, d=''):
    return params_init(self, d or params_dir)

  with patch.object(Params, '__init__', isolated_params):
    import pyray as rl
    from opendbc.car import structs
    from openpilot.cereal import log, messaging
    from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
    from openpilot.selfdrive.ui.bp.mici.onroad.torque_bar_bp import TorqueBarBP
    from openpilot.selfdrive.ui.bp.onroad.torque_bar_renderer_bp import TorqueBarRendererBP
    from openpilot.system.ui.lib.application import gui_app

    class Messages(dict):
      valid = dict.fromkeys(('controlsState', 'carState', 'carControl', 'vehicleParameters', 'carOutput', 'carStateBP'), True)
      alive = valid.copy()

    cs = log.ControlsState.new_message()
    car = structs.CarState.new_message(vEgo=20.)
    cc = structs.CarControl.new_message(latActive=True)
    params = log.VehicleParameters.new_message(roll=0.)
    output = messaging.new_message('carOutput').carOutput
    ford = messaging.new_message('carStateBP').carStateBP
    cp = structs.CarParams.new_message(brand='ford', maxLateralAccel=1.5)
    sm = Messages()
    gui_app.init_window('Steering arc regression test')
    try:
      rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
      with patch.object(ui_state, 'sm', sm), patch.object(ui_state, 'CP', cp.as_reader()), \
           patch.object(ui_state, 'status', UIStatus.ENGAGED), patch.object(ui_state, 'torque_bar', True):
        for cls in (TorqueBarBP, TorqueBarRendererBP):
          widget = cls()
          def update(demand, roll=0., kind='angleState', active=True, current_widget=widget):
            cs.init('lateralControlState').init(kind)
            cs.desiredCurvature = demand / car.vEgo ** 2
            params.roll = roll
            cc.latActive = active
            sm.update(controlsState=cs.as_reader(), carState=car.as_reader(), carControl=cc.as_reader(),
                      vehicleParameters=params.as_reader(), carOutput=output.as_reader(), carStateBP=ford.as_reader())
            current_widget._update_torque_filter_bp()
            return current_widget._torque_filter.x

          for kind in ('angleState', 'curvatureState'):
            for sign in (-1., 1.):
              widget._torque_filter.x = 0.
              # A bank can reduce estimated effort, but cannot reverse the turn arrow.
              value = update(sign * .04, sign * .06, kind)
              assert math.isfinite(value) and value * sign >= 0., (cls, kind, sign, value)
              for _ in range(100):
                value = update(sign * .75, 0., kind)
              assert abs(value - sign * .5) < .001, (cls, kind, value)
              # Do not keep showing the previous direction after demand reverses.
              assert update(-sign * .75, 0., kind) * -sign > 0.
              for _ in range(100):
                value = update(sign * 10., 0., kind)
              assert .99 <= abs(value) <= 1.
              assert update(0., 0., kind) == 0.
              assert update(sign * .75, 0., kind, active=False) == 0.

          # Invalid, dead or non-finite telemetry must clear the display, not freeze it.
          for collection in (sm.valid, sm.alive):
            for service in ('controlsState', 'carState', 'carControl', 'vehicleParameters'):
              update(.75)
              collection[service] = False
              assert update(.75) == 0.
              assert not widget._bp_torque_valid
              collection[service] = True
          assert update(float('nan')) == 0. and not widget._bp_torque_valid
          for maximum in (0., -1., float('nan'), float('inf')):
            cp.maxLateralAccel = maximum
            assert update(.75) == 0. and not widget._bp_torque_valid
          cp.maxLateralAccel = 1.5
          output.actuatorsOutput.torque = -.4
          for _ in range(100):
            value = update(.75, kind='pidState')
          assert abs(value - .4) < .001
          assert update(.75) > 0. and widget._bp_torque_valid
          # Reported limit colors are immediate and independent of demand magnitude,
          # turn direction, lateral-control-state variant, smoothing, or accent colors.
          feedback = ford.fordSteeringLimit
          feedback.dataAvailable = True
          feedback.controlStatus = 2
          for kind in ('angleState', 'curvatureState'):
            for sign in (-1., 1.):
              for status, expected in ((0, (230, 230, 230)), (1, (255, 200, 0)), (2, (255, 65, 65)), (3, (255, 65, 65))):
                feedback.status = status
                feedback.sourceMonoTime = time.monotonic_ns()
                update(sign * .05, kind=kind)
                color, _ = widget._torque_colors(rl.Color(0, 0, 255, 255), rl.Color(0, 255, 0, 255))
                assert (color.r, color.g, color.b) == expected
                assert widget._torque_filter.x * sign > 0
          # Losing only the feedback leaves direction/demand visible, with unknown capacity.
          sm.alive['carStateBP'] = False
          update(10.)
          assert widget._bp_torque_valid and widget._bp_limit_display.label == 'DEMAND / CAPACITY UNKNOWN'
          color, _ = widget._torque_colors(rl.Color(255, 0, 0, 255), rl.Color(255, 0, 0, 255))
          assert (color.r, color.g, color.b) == (230, 230, 230)
          sm.alive['carStateBP'] = True
          feedback.sourceMonoTime = time.monotonic_ns() - 200_000_000
          update(.75)
          assert widget._bp_limit_display.label == 'DEMAND / CAPACITY UNKNOWN'
          assert update(.75, active=False) == 0 and widget._bp_limit_display.label == ''
          update(.75)
          for status in (0, 1, 2, 3):
            feedback.status = status
            feedback.sourceMonoTime = time.monotonic_ns()
            for _ in range(80):
              update(.75)
              if isinstance(widget, TorqueBarRendererBP):
                widget.update()
              else:
                widget._torque_line_alpha_filter.update(1.)
            rl.begin_drawing()
            try:
              rl.clear_background(rl.BLACK)
              widget.render(rect)
              if isinstance(widget, TorqueBarRendererBP):
                widget.render_strip(rl.Rectangle(30, 150, 300, 11))
                widget.render_strip_arched(rect, 250, 420, -90, -150, -30, 150)
            finally:
              rl.end_drawing()
            if screenshot_dir := os.environ.get('BP_ARC_SCREENSHOT_DIR'):
              screenshot = rl.load_image_from_screen()
              try:
                assert rl.export_image(screenshot, str(Path(screenshot_dir) / f'{cls.__name__}-{status}.png'))
              finally:
                rl.unload_image(screenshot)
    finally:
      gui_app.close()


if __name__ == '__main__':
  render_check(sys.argv[1])
