"""
BluePilot Ford longitudinal follow control extension.

Implements smoother highway following by classifying lead vehicle behavior
(gaining, pacing, trailing) and applying gas limits per state. Also
adds split brake/precharge hysteresis for smoother deceleration.

Key features:
  - Speed deadband: BP long engages above 50 mph, disengages below 45 mph
  - Lead classification: gaining (closing in), pacing (matching), trailing (falling behind)
  - Gas limits per state: zero gas when gaining within 1.5s, capped gas when pacing
  - Preserve the upstream acceleration request, including braking with no lead
  - Fall back to upstream control when lead data is absent or unhealthy
  - Mutual exclusion: brake_actuate forces gas to INACTIVE_GAS
"""

from collections import namedtuple
from math import isfinite

from numpy import clip

from opendbc.car.ford.values import CarControllerParams


# Result namedtuple returned by LongitudinalExt.update()
LongitudinalResult = namedtuple('LongitudinalResult', [
  'accel',
  'gas',
  'brake_actuate',
  'precharge_actuate',
  'accel_pred_send',
  'stopping',
  'target_speed',
  'bp_long_used',
])


class LongitudinalExt:
  """
  BluePilot longitudinal follow control extension for Ford vehicles.

  Mixed into CarController via multiple inheritance. The stock carcontroller
  computes op_accel/op_gas using upstream logic, then calls
  LongitudinalExt.update() to apply BP follow control on top.

  The SubMaster (for radarState) is owned by LateralCurvExt and shared via self.sm
  since both classes are mixed into the same CarController instance.
  """

  def __init__(self, CP, CP_SP):
    # BP longitudinal state
    self._bp_long_active_last = False
    self.bp_gas_last = 0.0
    self.bp_accel_last = 0.0
    self.bpSpeedAllow = False

    # Thresholds
    self.MAX_URBAN_SPEED_MPH = 45.0

    # Brake hysteresis thresholds
    self.brake_actuate_target = -0.14   # engage brakes below this accel
    self.brake_actuate_release = -0.06  # release brakes above this accel
    self.precharge_actuate_target = -0.12
    self.precharge_actuate_release = -0.06
    self.op_brake_actuate_last = False
    self.precharge_actuate_last = False

    # Toggles (updated from Params each frame)
    self.disable_BP_long_UI = False
    self.disable_downhill_comp_UI = True

  def update_long_params(self, params):
    """Read longitudinal-related Params from the UI. Called each frame."""
    self.disable_BP_long_UI = params.get_bool("disable_BP_long_UI")
    self.disable_downhill_comp_UI = params.get_bool("disable_downhill_comp_UI")

  def update(self, CC, CS, op_accel, op_gas, accel_due_to_pitch, v_ego_mph, stopping, target_speed):
    """
    Apply BluePilot longitudinal follow control on top of stock op_accel/op_gas.

    Called at 50Hz from CarController.update() inside the ACC_CONTROL_STEP block,
    after stock creep compensation and rate limiting have been applied.

    Args:
      CC: CarControl with longActive
      CS: CarState with vEgo, gasPressed, brakePressed
      op_accel: Stock openpilot accel after creep comp + rate limit (m/s^2)
      op_gas: Stock openpilot gas value (m/s^2)
      accel_due_to_pitch: Pitch compensation value (m/s^2, may be clamped by downhill toggle)
      v_ego_mph: Current speed in mph
      stopping: True if in stopping state
      target_speed: Target cruise speed (km/h)

    Returns:
      LongitudinalResult namedtuple with final accel, gas, brake, precharge values.
    """
    # The caller already applies the upstream brake slew limit. This extension must
    # never relax that acceleration request, even when radar loses the lead.
    accel = op_accel
    gas = op_gas
    control_active = CC.longActive and not CS.out.gasPressed and not CS.out.brakePressed

    # Speed deadband: engage above 50 mph, disallow below 45 mph. Do not carry
    # eligibility from a previous engagement or across driver intervention.
    if not control_active or self.disable_BP_long_UI or v_ego_mph < self.MAX_URBAN_SPEED_MPH:
      self.bpSpeedAllow = False
    elif v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5:
      self.bpSpeedAllow = True

    bp_long_used = False
    if control_active and not self.disable_BP_long_UI and self.bpSpeedAllow:
      # SubMaster alive includes message-age checks. Do not require updated here:
      # radar runs slower than this 50 Hz consumer, so healthy inter-message ticks
      # should keep using the current lead. A valid but dead publisher must fall back.
      if self.sm.valid.get('radarState', False) and self.sm.alive.get('radarState', False):
        lead = getattr(self.sm['radarState'], 'leadOne', None)
        # Current cereal uses present; support older donor schemas without letting
        # their deprecated status override an explicit present=False.
        lead_present = lead is not None and getattr(lead, 'present', getattr(lead, 'status', False))
        if lead_present:
          d_rel = float(getattr(lead, 'dRel', float('nan')))
          v_rel = float(getattr(lead, 'vRel', float('nan')))
          v_lead = float(getattr(lead, 'vLead', float('nan')))
          if all(isfinite(v) for v in (d_rel, v_rel, v_lead, CS.out.vEgo)) and d_rel > 0 and v_lead * 2.23694 > 40.0:
            bp_long_used = True
            lead_time_sec = d_rel / max(CS.out.vEgo, 0.5)
            # These are upper caps only. Preserve negative gas requests and the
            # inactive sentinel instead of raising either to zero when following.
            if gas != CarControllerParams.INACTIVE_GAS:
              if v_rel < -0.1 and lead_time_sec < 1.5:
                gas = min(gas, 0.0)
              elif -0.1 <= v_rel <= 0.1:
                gas = min(gas, max(0.0, 0.2 + accel_due_to_pitch))

    # Use the actual, pitch-compensated upstream demand for both latches. Keeping
    # the returned state across frames preserves hysteresis inside the deadband,
    # including transitions between BP gas caps and upstream-only operation.
    accel_pitch_compensated = accel + accel_due_to_pitch
    brake_actuate = self.op_brake_actuate_last
    if not control_active or accel_pitch_compensated > self.brake_actuate_release:
      brake_actuate = False
    elif accel_pitch_compensated < self.brake_actuate_target:
      brake_actuate = True

    precharge_actuate = self.precharge_actuate_last
    if not control_active or accel_pitch_compensated > self.precharge_actuate_release:
      precharge_actuate = False
    elif not bp_long_used:
      precharge_actuate = brake_actuate
    elif accel_pitch_compensated < self.precharge_actuate_target:
      precharge_actuate = True

    # Mutual exclusion: no brake and gas at the same time
    if brake_actuate or not CC.longActive:
      gas = CarControllerParams.INACTIVE_GAS

    # Clip to ford.h ACCDATA safety limits
    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
    accel_pred_send = CarControllerParams.INACTIVE_GAS

    self._bp_long_active_last = bp_long_used
    self.op_brake_actuate_last = brake_actuate
    self.precharge_actuate_last = precharge_actuate
    self.bp_accel_last = accel
    self.bp_gas_last = gas

    return LongitudinalResult(
      accel=accel,
      gas=gas,
      brake_actuate=brake_actuate,
      precharge_actuate=precharge_actuate,
      accel_pred_send=accel_pred_send,
      stopping=stopping,
      target_speed=target_speed,
      bp_long_used=bp_long_used,
    )
