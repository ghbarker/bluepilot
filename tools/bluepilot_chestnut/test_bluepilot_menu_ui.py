import os
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
          for item in panel._scroller.items:
            # Exercise controls beyond the initially visible scroller viewport.
            render(item, rl.Rectangle(0, 0, item.rect.width, item.rect.height))
          panel.hide_event()
          panel.show_event()
          render(panel, rect)
          panel.hide_event()

        assert not params.get_bool('DoReboot'), 'Opening settings must not request a reboot'
      finally:
        gui_app.close()


if __name__ == '__main__':
  render_check(sys.argv[1], sys.argv[2], sys.argv[3])
