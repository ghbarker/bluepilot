import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from openpilot.common.params import Params

# BluePilot: extension imports for lateral, longitudinal, and HUD control
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt, PrimaryLateralControl
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalExt
from opendbc.sunnypilot.car.ford.hud_ext import HudExt
from opendbc.sunnypilot.car.ford import fordcan_ext
from opendbc.sunnypilot.car.ford.chestnut_compat import limit_curvature_for_chestnut
from opendbc.sunnypilot.car.ford.icbm import IntelligentCruiseButtonManagementInterface

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


# BluePilot: CarController inherits from LateralCurvExt, LateralAngleExt, LongitudinalExt, HudExt,
# and ICBM for 4-signal lateral control (curvature- or angle-primary), follow-aware longitudinal,
# and enhanced HUD messaging.
# Init order: CarControllerBase first (sets self.CP, self.frame), then ext classes.
class CarController(CarControllerBase, LateralCurvExt, LateralAngleExt, LongitudinalExt, HudExt,
                    IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    # BluePilot: initialize extension classes
    LateralCurvExt.__init__(self, CP, CP_SP)
    LateralAngleExt.__init__(self, CP, CP_SP)
    LongitudinalExt.__init__(self, CP, CP_SP)
    HudExt.__init__(self, CP, CP_SP)
    # ICBM: base class sets state used at runtime, init for robustness
    # IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)

    self.params = Params()
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.bp_lateral_mode_last = None
    self.anti_overshoot_curvature_last = 0
    self.disable_BP_lat_UI = False
    self.accel = 0.0
    self.gas = 0.0
    self.last_button_frame = 0  # BluePilot: ICBM button press tracking
    # Note: main_on_last, lkas_enabled_last, steer_alert_last, lead_distance_bars_last,
    # distance_bar_frame are initialized by HudExt.__init__() above

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # BluePilot: update SubMaster (modelV2, liveParameters, selfdriveState, radarState) and vehicle model
    LateralCurvExt.update_sm(self)

    # BluePilot: read runtime params from UI
    LateralCurvExt.update_lateral_params(self, self.params)
    LateralAngleExt.update_angle_params(self, self.params)
    self.disable_BP_lat_UI = self.params.get_bool("disable_BP_lat_UI")
    LongitudinalExt.update_long_params(self, self.params)
    HudExt.update_hud_params(self, self.params, self.CP)

    actuators = CC.actuators
    hud_control = CC.hudControl
    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # BluePilot: compute DM state (TJA message, warning, hands level)
    HudExt.update_dm(self, hud_control, main_on, CS.out.cruiseState.standstill, self.frame)

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    # BluePilot: Intelligent Cruise Button Management (ICBM)
    icbm_can_sends, self.last_button_frame = IntelligentCruiseButtonManagementInterface.update(
      self, CC_SP, CS, self.packer, self.CAN, self.frame, self.last_button_frame
    )
    can_sends.extend(icbm_can_sends)

    ### lateral control ###
    # BluePilot: keep stock lateral path in carcontroller, and run BP 4-signal lateral
    # only when bypass is disabled.
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
      lateral_mode = "stock" if self.disable_BP_lat_UI else self.primary_lateral_control
      mode_changed = lateral_mode != self.bp_lateral_mode_last
      self.bp_lateral_mode_last = lateral_mode
      # BluePilot: bypass uses current upstream curvature control.
      if self.disable_BP_lat_UI:
        stock_lat_active = CC.latActive and not mode_changed
        apply_curvature = limit_curvature_for_chestnut(
          actuators.curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw, stock_lat_active)
        self.apply_curvature_last = apply_curvature

        if self.CP.flags & FordFlags.CANFD:
          mode = 1 if stock_lat_active else 0
          counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))
        else:
          can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, stock_lat_active, 0., 0., -self.apply_curvature_last, 0.))
      else:
        # BluePilot: select the BP lateral strategy by primary control variable.
        #   1 (angle)     -> LateralAngleExt: κ → path_angle (c1), apply_curvature held at 0.
        #   0 (curvature) -> LateralCurvExt: full 4-signal curvature-primary (default).
        # Both return a LateralResult packed identically below.
        # Do not run apply_ford_curvature_limits here or overwrite apply_curvature_last before the
        # strategy runs. Panda rate-checks desired_curvature vs the last TX on the bus; that must match
        # the prior frame's lat.apply_curvature only (not an intermediate stock-limited value).
        strategy_control = CC
        if mode_changed:
          strategy_control = CC.as_builder()
          strategy_control.latActive = False
        if self.primary_lateral_control == PrimaryLateralControl.angle:
          lat = LateralAngleExt.update_angle_strategy(self, strategy_control, CS, actuators, self.CP)
        else:
          lat = LateralCurvExt.update(self, strategy_control, CS, actuators, self.apply_curvature_last, self.CP)
        self.lateralUncertainty = lat.lateralUncertainty
        # BluePilot: rate-limit diagnostics for controllerStateBP. update_angle_strategy sets these on
        # self (angle mode); curvature mode leaves them False (the path_angle ROC / sim aren't run there).
        _angle_mode = self.primary_lateral_control == PrimaryLateralControl.angle
        self.angleRateLimited = getattr(self, 'bp_angle_rate_limited', False) if _angle_mode else False
        self.curvatureRateLimited = getattr(self, 'bp_curvature_rate_limited', False) if _angle_mode else False
        # BluePilot: current-curvature deviation-clip diagnostic. Set by whichever strategy just ran
        # (both lateral_curv_ext.update and lateral_angle_ext.update_angle_strategy set this), so it's
        # meaningful in both modes -- not gated by _angle_mode like the two above.
        self.curvatureDeviationLimited = getattr(self, 'bp_curvature_deviation_limited', False)
        self.humanTurnLateralPaused = self.angle_human_turn_active if _angle_mode else False
        self.stallBlipActive = self.angle_stall_blip_active if _angle_mode else False

        # BluePilot: angle-mode human-turn override -- send lateral inactive (mode 0) while the
        # driver manually turns, so the PSCM releases cleanly instead of stalling 2-3 s on
        # re-engage (observed on Mach-E). Panda-clean: every ford.h check has a legitimate
        # !steer_control_enabled branch for the zeroed frames; on release, path_angle ramps back
        # from 0 through the soft ROC (no reset-bypass latch involvement). Curvature mode also sends inactive during its reset_steering path.
        # The stall blip (lateral_angle_ext.py) rides the same mode-0 path: a short pulse that
        # resets the PSCM's post-override attenuation when the deviation clip deadlocks hands-free.
        # Reset frames disengage lateral actuation instead of relying on the
        # donor's removed three-second safety bypass. A mode change also sends
        # one neutral frame before enabling the new actuator representation.
        strategy_paused = ((self.angle_human_turn_active or self.angle_stall_blip_active) if _angle_mode
                           else self.reset_steering_last)
        lat_active = CC.latActive and not (strategy_paused or mode_changed)
        if not lat_active:
          lat = lat._replace(apply_curvature=0., path_angle=0., path_offset=0., curvature_rate=0.)
        elif not _angle_mode:
          # The donor computes its four signals unchanged. Adapt its final wire
          # curvature to the current base's envelope, then retain that TX value.
          limited = limit_curvature_for_chestnut(lat.apply_curvature, self.apply_curvature_last,
                                                 self.get_current_curvature(CS), CS.out.vEgoRaw, True)
          self.curvatureRateLimited = abs(limited - lat.apply_curvature) > 1e-9
          lat = lat._replace(apply_curvature=limited)
        self.apply_curvature_last = lat.apply_curvature
        if self.CP.flags & FordFlags.CANFD:
          mode = 1 if lat_active else 0
          counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
          can_sends.append(fordcan_ext.create_lat_ctl2_msg(
            self.packer, self.CAN, mode, lat.ramp_type, lat.precision_type,
            -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate, counter
          ))
        else:
          can_sends.append(fordcan_ext.create_lat_ctl_msg(
            self.packer, self.CAN, lat_active, lat.ramp_type, lat.precision_type,
            -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate
          ))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      # BluePilot: tell ford.h whether angle mode is engaged, out-of-band from LMC/LMC2, packed into
      # Lane_Assist_Data1's unused bits (read synchronously in ford_tx_hook, no separate CAN ID/RX
      # needed). bypass_bp_lat means BP lateral is off entirely, so angle mode can't be engaged then.
      # shadow_curvature is only meaningful in angle mode (self.bp_kappa_cmd is stale/unused CurvExt
      # state otherwise, so force it to 0 there).
      # Negated to match the sign convention path_angle/apply_curvature use on the wire (see
      # -lat.path_angle/-lat.apply_curvature just above) -- ford.h's angle_meas (measured curvature,
      # from raw yaw rate with no negation) is calibrated against that wire convention, not
      # bp_kappa_cmd's internal one. Confirmed via safety_replay against a real route (2026-07-10):
      # un-negated, shadow_curvature and angle_meas were consistently opposite-signed, so the
      # deviation check found a "divergence" on every frame once speed crossed angle_error_min_speed.
      angle_mode_engaged = (not self.disable_BP_lat_UI) and (self.primary_lateral_control == PrimaryLateralControl.angle)
      shadow_curvature = -self.bp_kappa_cmd if angle_mode_engaged else 0.0
      can_sends.append(fordcan_ext.create_lka_msg(
        self.packer, self.CAN, CC.latActive, hud_control, angle_mode_engaged, shadow_curvature
      ))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # Stock creep compensation and rate limiting (upstream-identical)
      op_accel = actuators.accel
      op_gas = op_accel

      if CC.longActive:
        op_accel = apply_creep_compensation(op_accel, CS.out.vEgo)
        op_accel = max(op_accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      op_accel = float(np.clip(op_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      op_gas = float(np.clip(op_gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      if not CC.longActive or op_gas < CarControllerParams.MIN_GAS:
        op_gas = CarControllerParams.INACTIVE_GAS

      # Pitch compensation (upstream-identical)
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      # BluePilot: downhill compensation disable
      if self.disable_downhill_comp_UI:
        if accel_due_to_pitch < 0:
          accel_due_to_pitch = 0

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      target_speed = V_CRUISE_MAX
      v_ego_mph = CS.out.vEgo * 2.23694

      # BluePilot: longitudinal follow control via LongitudinalExt
      # Classifies lead vehicle state (gaining/pacing/trailing) and applies gas/accel limits,
      # rate-limited braking, and split brake/precharge hysteresis.
      lng = LongitudinalExt.update(self, CC, CS, op_accel, op_gas, accel_due_to_pitch,
                                    v_ego_mph, stopping, target_speed)

      can_sends.append(fordcan_ext.create_acc_msg(
        self.packer, self.CAN, CC.longActive, lng.gas, lng.accel, lng.accel_pred_send,
        lng.stopping, lng.brake_actuate, lng.precharge_actuate, v_ego_kph=lng.target_speed
      ))

      self.accel = lng.accel
      self.gas = lng.gas

    ### ui ###
    # BluePilot: HUD message generation via HudExt
    # Handles LKAS UI (1Hz), ACC UI (5Hz), bar persistence, and TJA/hands-free messaging.
    hud_can_sends = HudExt.update_hud(self, CC, CS, hud_control, main_on, fcw_alert,
                                       self.frame, self.packer, self.CAN, self.CP)
    can_sends.extend(hud_can_sends)

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.gas)

    self.frame += 1
    return new_actuators, can_sends
