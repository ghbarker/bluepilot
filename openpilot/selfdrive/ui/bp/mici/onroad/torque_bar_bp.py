from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.bp.onroad.torque_bar_state_bp import TorqueBarStateBP


class TorqueBarBP(TorqueBarStateBP, TorqueBar):
  """BluePilot Mici torque bar with shared BP torque-state math.

  Rendering stays inherited from Mici, while the update math is shared with the
  COMMA_HARDWARE BP renderer.
  """

  def _update_state(self):
    if self._demo:
      return

    try:
      self._update_torque_filter_bp()
    except (KeyError, AttributeError):
      self._clear_torque_bp()

  def _render(self, rect):
    if self._demo or self._bp_torque_valid:
      super()._render(rect)
      if not self._demo:
        self._render_limit_label_bp(rect.x + rect.width / 2 + 8, rect.y + rect.height - 17 * self._scale,
                                    self._torque_line_alpha_filter.x, 12 * self._scale, rect.width - 20)
