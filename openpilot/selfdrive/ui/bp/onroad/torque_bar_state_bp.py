import math
import numpy as np
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.ui.ui_state import ui_state


DEFAULT_MAX_LAT_ACCEL_BP = 3.0  # m/s^2
ROLL_COMPENSATION_MAX_WEIGHT = 0.5


class TorqueBarStateBP:
  """Shared BluePilot torque bar state update math.

  Renderers stay separate between COMMA_HARDWARE and MICI, but the torque estimate should
  stay in one BP-owned place so upstream math changes are easier to absorb.
  """

  roll_compensation_max_weight = ROLL_COMPENSATION_MAX_WEIGHT
  default_max_lateral_accel = DEFAULT_MAX_LAT_ACCEL_BP
  _bp_torque_valid = False

  def _clear_torque_bp(self) -> None:
    self._bp_torque_valid = False
    self._torque_filter.x = 0.0

  def _update_torque_filter_bp(self) -> None:
    sm = ui_state.sm
    angle_control = sm['controlsState'].lateralControlState.which() in ('angleState', 'curvatureState')
    services = ('controlsState', 'carState', 'carControl', 'vehicleParameters' if angle_control else 'carOutput')
    if not all(sm.valid[s] and sm.alive[s] for s in services):
      self._clear_torque_bp()
      return

    if not sm['carControl'].latActive:
      target = 0.0
    elif angle_control:
      speed = sm['carState'].vEgo
      demand = sm['controlsState'].desiredCurvature * speed ** 2
      roll = sm['vehicleParameters'].roll
      maximum = ui_state.CP.maxLateralAccel if ui_state.CP else self.default_max_lateral_accel
      if not all(math.isfinite(v) for v in (speed, demand, roll, maximum)) or maximum <= 0.:
        self._clear_torque_bp()
        return
      # This is an estimated load, not measured EPS torque or remaining firmware
      # authority. Bank compensation may reduce effort, but must not point the
      # directional arc opposite the requested turn (or invent a turn at zero).
      compensation = roll * ACCELERATION_DUE_TO_GRAVITY * self._roll_compensation_weight_bp(speed)
      load = max(0., abs(demand) - math.copysign(1., demand) * compensation)
      target = math.copysign(load / maximum, demand) if demand != 0. else 0.0
    else:
      target = -sm['carOutput'].actuatorsOutput.torque

    if not math.isfinite(target):
      self._clear_torque_bp()
      return
    target = float(np.clip(target, -1., 1.))
    # Smoothing must not keep displaying the old side after a reversal or release.
    if target == 0. or target * self._torque_filter.x < 0.:
      self._torque_filter.x = 0.0
    self._torque_filter.update(target)
    self._bp_torque_valid = True

  def _roll_compensation_weight_bp(self, v_ego: float) -> float:
    # Roll is less accurate near standstill, so reduce its effect at low speed.
    # Full roll compensation makes crowned straightaways look like steering bias.
    return np.interp(v_ego, [5, 15], [0.0, self.roll_compensation_max_weight])
