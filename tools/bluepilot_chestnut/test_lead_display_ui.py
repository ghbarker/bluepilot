import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('metric', ['0', '1'])
def test_mici_lead_displays_render_current_radar_messages(tmp_path, metric):
  result = subprocess.run(['xvfb-run', '-a', sys.executable, str(Path(__file__).resolve()), str(tmp_path / 'params'), metric],
                          env={**os.environ, 'BIG': '0', 'SCALE': '1'}, capture_output=True, text=True, timeout=40)
  assert result.returncode == 0, result.stdout + result.stderr


def render_check(params_dir, metric):
  from types import SimpleNamespace
  from unittest.mock import MagicMock, patch
  from openpilot.common.params import Params

  params_init = Params.__init__
  def isolated_params(self, d=''):
    return params_init(self, d or params_dir)

  with patch.object(Params, '__init__', isolated_params):
    import pyray as rl
    import numpy as np
    from opendbc.car import structs
    from openpilot.cereal import log
    from openpilot.common.constants import CV
    from openpilot.system.ui.lib.application import gui_app
    from openpilot.selfdrive.ui.bp.mici.onroad import complication

    class Messages(dict):
      valid = {'radarState': True, 'carState': True, 'longitudinalPlanSP': False}

    radar = log.RadarState.new_message()
    car = structs.CarState.new_message(gearShifter='drive', vEgoCluster=20.)
    sm = Messages()
    now = [0.5]  # Exercise startup before the fade interval has elapsed, too.
    params = Params()
    gui_app.init_window('Lead display regression test')
    try:
      rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
      with patch.object(complication.ui_state, 'sm', sm), patch.object(complication.ui_state, 'is_metric', metric), \
           patch.object(complication, 'time', SimpleNamespace(monotonic=lambda: now[0])):
        def render(widget, mode, present=False, valid=True, gear='drive', vrel=2.):
          params.put('mici_complication', mode, block=True)
          radar.leadOne.present = present
          radar.leadOne.dRel = 40.
          radar.leadOne.vRel = vrel
          car.gearShifter = gear
          sm['carState'], sm['radarState'] = car.as_reader(), radar.as_reader()
          sm.valid['radarState'] = valid
          with patch.object(rl, 'draw_text_ex', wraps=rl.draw_text_ex) as draw:
            rl.begin_drawing()
            try:
              rl.clear_background(rl.BLACK)
              widget.render(rect)
            finally:
              rl.end_drawing()
          return [c.args[1] for c in draw.call_args_list]

        # The enabled display must read real LeadData.present, including no lead.
        for mode in range(5):
          widget = complication.MiciComplication()
          text = render(widget, mode)
          if mode != 2:
            assert text == [], (mode, text)
          text = render(widget, mode, present=True)
          if mode == 0:
            assert text == []
          else:
            assert text, mode
          if mode == 1:
            conversion = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
            assert str(round(22. * conversion)) in text
            assert ('km/h' if metric else 'mph') in text
          elif mode == 3:
            assert str(round(40. if metric else 40. * 3.28084)) in text
            assert ('m' if metric else 'ft') in text
          elif mode == 4:
            assert '2.0' in text and 'sec' in text

        # Lead-speed colors, stationary leads, and negative computed speed.
        for vrel in (-25., -2., 0., 2.):
          widget = complication.MiciComplication()
          text = render(widget, 1, present=True, vrel=vrel)
          conversion = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
          assert str(round(max(0., (20. + vrel) * conversion))) in text

        # Existing values fade after loss; fresh modes cannot reuse another
        # display's timer or uninitialized distance/headway cache.
        for mode in (1, 3, 4):
          widget = complication.MiciComplication()
          now[0] = 100.
          assert render(widget, mode, present=True)
          now[0] += 1.
          assert render(widget, mode, present=False)
          now[0] += complication.DELAY
          assert render(widget, mode, present=False) == []
          for next_mode in (1, 3, 4):
            now[0] += 10.
            assert render(widget, 1, present=True)
            if next_mode != 1:
              assert render(widget, next_mode, present=False) == []

        for mode in (1, 3, 4):
          for gear in ('park', 'reverse'):
            assert render(complication.MiciComplication(), mode, present=True, gear=gear) == []
          assert render(complication.MiciComplication(), mode, present=True, valid=False) == []

        # Other BP consumers of the same renamed radar flag must accept the
        # current schema too. These checks exercise their lead-processing paths.
        from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
        from openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp import ChevronMetricsBP
        from openpilot.selfdrive.ui.bp.onroad.rad_racer_theme import RadRacerTheme
        from bluepilot.ui.widgets.debug.other_debug_panel import OtherDebugPanel

        renderer = ModelRendererBP()
        renderer._path.raw_points = np.array([[0., 0., 0.], [100., 0., 0.]])
        renderer._camera_offset = renderer._path_offset_z = 0.
        renderer._map_to_screen = MagicMock(return_value=(100., 100.))
        renderer._update_lead_vehicle = MagicMock(return_value=SimpleNamespace(chevron=[(100., 100.), (110., 120.), (90., 120.)]))
        metrics = ChevronMetricsBP()
        theme = RadRacerTheme()
        debug_panel = OtherDebugPanel()
        torque_bar = SimpleNamespace(_torque_filter=SimpleNamespace(x=0.))
        sm.valid['radarState'] = True

        for present_one, present_two in ((False, False), (True, False), (False, True), (True, True)):
          radar.leadOne.present, radar.leadTwo.present = present_one, present_two
          radar.leadOne.radar, radar.leadTwo.radar = True, False
          radar.leadTwo.dRel = 60.
          sm['radarState'] = radar.as_reader()
          renderer._update_leads(sm['radarState'], renderer._path.raw_points[:, 0])
          renderer._update_lead_radar_status(sm['radarState'])
          assert renderer._lead_was_active == [present_one, present_two]
          assert renderer._lead_is_radar == [present_one, False]
          with patch.object(metrics, 'should_render', return_value=True), patch.object(metrics, '_draw_lead') as draw_lead:
            metrics.draw_lead_status(sm, sm['radarState'], rect, renderer._lead_vehicles)
            assert draw_lead.call_count == int(present_one) + int(present_two)
          debug_panel._update_radar(sm)
          assert ('Status', 'Tracking' if present_one else 'No Lead') in debug_panel._tab_cards[1][0]._rows
          assert ('Status', 'Tracking' if present_two else 'No Lead') in debug_panel._tab_cards[1][1]._rows
          rl.begin_drawing()
          try:
            theme.render_foreground(rect, renderer, theme.cluster_top(rect))
            theme.render_cluster(rect, torque_bar)
          finally:
            rl.end_drawing()
        assert not params.get_bool('DoReboot')
    finally:
      gui_app.close()


if __name__ == '__main__':
  render_check(sys.argv[1], sys.argv[2] == '1')
