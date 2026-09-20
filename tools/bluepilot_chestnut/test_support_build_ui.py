import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('big', ['0', '1'])
def test_build_error_display_and_touch_scroll(big):
  result = subprocess.run(['xvfb-run', '-a', '--server-args=-screen 0 2400x1400x24',
                           sys.executable, str(Path(__file__).resolve())],
                          env={**os.environ, 'BIG': big}, capture_output=True, text=True, timeout=40)
  assert result.returncode == 0, result.stdout + result.stderr


def render_check():
  from unittest.mock import patch
  import pyray as rl
  from openpilot.system.ui.lib.application import gui_app, MouseEvent, MousePos
  from openpilot.system.ui.bp_spinner import BPSpinner

  gui_app.init_window('Build screen test')
  screen = BPSpinner()
  rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
  for i in range(40):
    screen.set_text(f'30|Build line {i}')
  screen.set_text('30|' + '/long-build-path' * 12)
  screen.set_text('30|Final compiler error detail')
  screen.set_text('BUILD_FAILED')

  with patch.object(rl, 'draw_text_ex', wraps=rl.draw_text_ex) as draw:
    rl.begin_drawing()
    screen.render(rect)
    rl.end_drawing()
    rendered = [call for call in draw.call_args_list if call.args[1] == 'Final compiler error detail']
    assert rendered, 'The compiler error must be drawn on both display sizes'
    line = rendered[0]
    assert 0 < line.args[2].y < screen._reboot_button_rect.y
    for call in draw.call_args_list:
      text, position, size = call.args[1:4]
      assert position.x >= 0
      assert rl.measure_text_ex(gui_app.font(), text, size, 0).x + position.x <= gui_app.width + 1

  tail = screen._scroll_offset
  assert tail > 0
  screen._handle_mouse_event(MouseEvent(MousePos(30, 70), 0, True, False, True, 0.))
  screen._handle_mouse_event(MouseEvent(MousePos(30, 100), 0, False, False, True, 0.1))
  assert screen._scroll_offset < tail
  rl.begin_drawing()
  screen.render(rect)
  rl.end_drawing()
  assert screen._scroll_offset < tail, 'Reading older lines must not snap back to the tail'
  button = screen._reboot_button_rect
  release = MousePos(button.x + 5, button.y + 5)
  screen._handle_mouse_event(MouseEvent(release, 0, False, True, False, 0.2))
  with patch('openpilot.system.ui.bp_spinner.subprocess.run') as reboot:
    screen._handle_mouse_release(release)
    reboot.assert_not_called()  # scrolling onto the button is not a reboot tap
  screen.set_text('BUILD_RETRY')
  assert not screen._error_mode and screen._scroll_offset == 0


if __name__ == '__main__':
  render_check()
