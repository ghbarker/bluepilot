import os
import json
import time
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('saved_settings', ['fresh', 'configured'])
@pytest.mark.parametrize('hardware', ['mici', 'tici'])
def test_bluepilot_menu_opens_renders_and_reopens(tmp_path, saved_settings, hardware):
  result = subprocess.run(['xvfb-run', '-a', sys.executable, str(Path(__file__).resolve()),
                           str(tmp_path / 'params'), saved_settings, hardware],
                          env={**os.environ, 'BIG': '0', 'SCALE': '1'},
                          capture_output=True, text=True, timeout=40)
  assert result.returncode == 0, result.stdout + result.stderr


def render_check(params_dir, saved_settings, hardware):
  from unittest.mock import patch
  from openpilot.common.params import Params

  params_init = Params.__init__

  def isolated_params(self, d=''):
    return params_init(self, d or params_dir)

  # UI singletons also need the isolated parameter store, before their imports.
  with patch.object(Params, '__init__', isolated_params):
    import pyray as rl
    from openpilot.system.ui.lib.application import gui_app
    from openpilot.system.ui.lib.wifi_manager import WifiManager
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.cereal import messaging
    from openpilot.selfdrive.ui.bp.mici.layouts.settings.bluepilot import (
      BluePilotLayoutMici, VehicleLayoutMici, AudioLayoutMici, VisualsLayoutMici,
      LateralLayoutMici, LongitudinalLayoutMici,
    )

    params = Params()
    if saved_settings == 'configured':
      params.put('WifiFavoriteSSID', 'A saved network with a long name')
      params.put('FordPrefLateralControl', 1)
      params.put_bool('EnableWebRoutesServer', True)
      params.put_bool('BPUIDebugLog', True)

    # Render real widgets without querying or changing the host's Wi-Fi.
    with patch.object(WifiManager, '_initialize'), patch.object(WifiManager, '_init_wifi_state'), \
         patch.object(WifiManager, '_update_networks'):
      gui_app.init_window('Bluepilot menu regression test')
      try:
        rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)

        def render(widget, area):
          rl.begin_drawing()
          try:
            rl.clear_background(rl.BLACK)
            widget.render(area)
          finally:
            rl.end_drawing()

        if hardware == 'tici':
          from openpilot.selfdrive.ui.bp.layouts.settings.bluepilot import BluePilotLayout
          panels = [BluePilotLayout()]
        else:
          panels = [BluePilotLayoutMici(back_callback=lambda: None), VehicleLayoutMici(), AudioLayoutMici(),
                    VisualsLayoutMici(), LateralLayoutMici(), LongitudinalLayoutMici()]
        for panel in panels:
          gui_app.push_widget(panel)
          render(panel, rect)  # Construction alone missed the first-frame crash.
          items = panel._scroller._items if hardware == 'tici' else panel._scroller.items
          for item in items:
            # Exercise controls beyond the initially visible scroller viewport.
            render(item, rl.Rectangle(0, 0, item.rect.width, item.rect.height))
          panel.hide_event()

          if isinstance(panel, LateralLayoutMici):
            params.put_bool('FordAngleAutoCal', True, block=True)
            params.put('FordAngleAutoCalState', '{"phase":"locked","pipe":{}}', block=True)
            item = panel.angle_autocal_status
            sm = ui_state.sm
            event = messaging.new_message('controllerStateBP', valid=True)
            statuses = [('delay', {'pause': 'delay'}, 'Waiting for steering delay'),
                        ('inactive', {'pause': 'inactive'}, 'Waiting for active steering'),
                        ('evidence', {'active': 'high', 'low': {'ph': 'good'},
                                      'high': {'ph': 'collect', 'reason': 'fit_evidence'}}, 'High-speed: more turn data needed'),
                        ('matching', {'low': {'ph': 'verify', 'reason': 'matching_turns'},
                                      'high': {'ph': 'collect'}}, 'Testing: matching turns needed')]
            for label, payload, expected in statuses:
              event.controllerStateBP.bmsAngleAutoCalState = json.dumps(payload)
              sm.update_msgs(time.monotonic(), [event.as_reader()])
              with patch.object(ui_state, 'started', True):
                item._next_refresh = 0.
                render(item, rl.Rectangle(0, 0, 402, 180))
                assert item.value == expected, (item.value, expected)
                assert not item._touch_valid()
                if screenshot_dir := os.environ.get('BP_AUTOCAL_SCREENSHOT_DIR'):
                  screenshot = rl.load_image_from_screen()
                  try:
                    assert rl.export_image(screenshot, str(Path(screenshot_dir) / f'autocal-{label}.png'))
                  finally:
                    rl.unload_image(screenshot)
            with patch.object(ui_state, 'started', True):
              sm.logMonoTime['controllerStateBP'] = time.monotonic_ns() - 3_000_000_000
              item._next_refresh = 0.
              item._update_state()
              assert item.value == 'Status unavailable'
            with patch.object(ui_state, 'started', False):
              item._next_refresh = 0.
              item._update_state()
              assert item.value == 'Saved: locked'
            assert params.get('FordAngleAutoCalState') == '{"phase":"locked","pipe":{}}'
          panel.show_event()
          render(panel, rect)
          panel.hide_event()

        assert not params.get_bool('DoReboot'), 'Opening settings must not request a reboot'
      finally:
        gui_app.close()


if __name__ == '__main__':
  render_check(sys.argv[1], sys.argv[2], sys.argv[3])
