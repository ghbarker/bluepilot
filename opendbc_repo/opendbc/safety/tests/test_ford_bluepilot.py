#!/usr/bin/env python3
import numpy as np
import random
import unittest

import opendbc.safety.tests.common as common
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from opendbc.safety.tests import test_ford as upstream_ford
from opendbc.car.ford.values import CAR, FordFlags, FordSafetyFlags
from opendbc.car.interfaces import scale_tire_stiffness
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.sunnypilot.car.ford.values_ext import FORD_PINION_GEOMETRY_INDEX, FORD_PINION_GEOMETRY_SHIFT, FordSafetyFlagsSP
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

MSG_EngBrakeData = 0x165           # RX from PCM, for driver brake pedal and cruise state
MSG_EngVehicleSpThrottle = 0x204   # RX from PCM, for driver throttle input
MSG_BrakeSysFeatures = 0x415       # RX from ABS, for vehicle speed
MSG_EngVehicleSpThrottle2 = 0x202  # RX from PCM, for second vehicle speed
MSG_Yaw_Data_FD1 = 0x91            # RX from RCM, for yaw rate
MSG_Steering_Data_FD1 = 0x083      # TX by OP, various driver switches and LKAS/CC buttons
MSG_ACCDATA = 0x186                # TX by OP, ACC controls
MSG_ACCDATA_3 = 0x18A              # TX by OP, ACC/TJA user interface
MSG_Lane_Assist_Data1 = 0x3CA      # TX by OP, Lane Keep Assist
MSG_LateralMotionControl = 0x3D3   # TX by OP, Lateral Control message
MSG_LateralMotionControl2 = 0x3D6  # TX by OP, alternate Lateral Control message
MSG_IPMA_Data = 0x3D8              # TX by OP, IPMA and LKAS user interface


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == MSG_Yaw_Data_FD1:
    chksum = dat[0] + dat[1]  # VehRol_W_Actl
    chksum += dat[2] + dat[3]  # VehYaw_W_Actl
    chksum += dat[5]  # VehRollYaw_No_Cnt
    chksum += dat[6] >> 6  # VehRolWActl_D_Qf
    chksum += (dat[6] >> 4) & 0x3  # VehYawWActl_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[4] = chksum

  elif addr == MSG_BrakeSysFeatures:
    chksum = dat[0] + dat[1]  # Veh_V_ActlBrk
    chksum += (dat[2] >> 2) & 0xf  # VehVActlBrk_No_Cnt
    chksum += dat[2] >> 6  # VehVActlBrk_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[3] = chksum

  elif addr == MSG_EngVehicleSpThrottle2:
    chksum = (dat[2] >> 3) & 0xf  # VehVActlEng_No_Cnt
    chksum += (dat[4] >> 5) & 0x3  # VehVActlEng_D_Qf
    chksum += dat[6] + dat[7]  # Veh_V_ActlEng
    chksum = 0xff - (chksum & 0xff)
    ret[1] = chksum

  return addr, ret, bus


class Buttons:
  CANCEL = 0
  RESUME = 1
  TJA_TOGGLE = 2


# Ford safety has four different configurations tested here:
#  * CAN with openpilot longitudinal
#  * CAN FD with stock longitudinal
#  * CAN FD with openpilot longitudinal

class TestFordSafetyBase(common.CarSafetyTest):
  # BluePilot: sunnypilot SP safety param (current_safety_param_sp), set before
  # set_safety_hooks in every concrete setUp -- ford_init reads it. 0 = stock behavior;
  # the pinion-curvature classes below override it (Toyota SAFETY_PARAM_SP convention).
  SAFETY_PARAM_SP: int = 0
  cnt_lat_ctl = 0
  LATERAL_FREQUENCY = 20

  STANDSTILL_THRESHOLD = 1
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_LateralMotionControl2, MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_LateralMotionControl2, MSG_IPMA_Data]}

  STEER_MESSAGE = 0

  # Curvature control limits
  DEG_TO_CAN = 50000  # 1 / (2e-5) rad to can
  MAX_CURVATURE = 0.02
  MAX_CURVATURE_ERROR = 0.002
  CURVATURE_ERROR_MIN_SPEED = 10.0  # m/s

  ANGLE_RATE_BP = [5., 25., 25.]
  ANGLE_RATE_UP = [0.00045, 0.0001, 0.0001]  # windup limit
  ANGLE_RATE_DOWN = [0.00045, 0.00015, 0.00015]  # unwind limit

  cnt_speed = 0
  cnt_speed_2 = 0
  cnt_yaw_rate = 0

  packer: CANPackerSafety
  safety: libsafety_py.LibSafety


  # The Chestnut curvature envelope applies with the BP flag enabled too.
  _get_max_curvature_can = upstream_ford.TestFordSafetyBase._get_max_curvature_can
  _get_max_curvature_delta_can = upstream_ford.TestFordSafetyBase._get_max_curvature_delta_can
  _get_max_curvature_delta_relaxed_can = upstream_ford.TestFordSafetyBase._get_max_curvature_delta_relaxed_can
  _get_max_curvature_relaxed_can = upstream_ford.TestFordSafetyBase._get_max_curvature_relaxed_can
  test_curvature_violation = upstream_ford.TestFordSafetyBase.test_curvature_violation
  test_rt_limits = upstream_ford.TestFordSafetyBase.test_rt_limits
  test_rx_hook_speed_mismatch = upstream_ford.TestFordSafetyBase.test_rx_hook_speed_mismatch

  def _set_prev_desired_angle(self, t):
    t = round(t * self.DEG_TO_CAN)
    self.safety.set_desired_angle_last(t)
    self.safety.set_desired_curvature_last(t)

  def _reset_curvature_measurement(self, curvature, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  # Driver brake pedal
  def _user_brake_msg(self, brake: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    enable = self.safety.get_controls_allowed()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # ABS vehicle speed
  def _speed_msg(self, speed: float, quality_flag=True):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3 if quality_flag else 0, "VehVActlBrk_No_Cnt": self.cnt_speed % 16}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values, fix_checksum=checksum)

  # PCM vehicle speed
  def _speed_msg_2(self, speed: float, quality_flag=True):
    # Ford relies on speed for driver curvature limiting, so it checks two sources
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3 if quality_flag else 0, "VehVActlEng_No_Cnt": self.cnt_speed_2 % 16}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values, fix_checksum=checksum)

  # Standstill state
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehStop_D_Stat": 1 if speed <= self.STANDSTILL_THRESHOLD else random.choice((0, 2, 3))}
    return self.packer.make_can_msg_safety("DesiredTorqBrk", 0, values)

  # Current curvature
  def _yaw_rate_msg(self, curvature: float, speed: float, quality_flag=True):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3 if quality_flag else 0,
              "VehRollYaw_No_Cnt": self.cnt_yaw_rate % 256}
    self.__class__.cnt_yaw_rate += 1
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values, fix_checksum=checksum)

  # Drive throttle input
  def _user_gas_msg(self, gas: float):
    values = {"ApedPos_Pc_ActlArb": gas}
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle", 0, values)

  # Cruise status
  def _pcm_status_msg(self, enable: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    brake = self.safety.get_brake_pressed_prev()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # LKAS command
  def _lkas_command_msg(self, action: int):
    values = {
      "LkaActvStats_D2_Req": action,
    }
    return self.packer.make_can_msg_safety("Lane_Assist_Data1", 0, values)

  # BluePilot: angle_mode_engaged + shadow_curvature, packed into Lane_Assist_Data1's unused bits
  # (byte4 bit0, bytes 5-6 -- see fordcan_ext.py's create_lka_msg / ford.h's FORD_Lane_Assist_Data1
  # tx_hook check). Sent by openpilot itself, so this goes through _tx, not _rx.
  def _lka_bp_status_msg(self, angle_mode_engaged: bool, shadow_curvature: float, action: int = 0):
    values = {"LkaActvStats_D2_Req": action}
    addr, dat, bus = self.packer.make_can_msg("Lane_Assist_Data1", 0, values)
    dat = bytearray(dat)
    shadow_curvature_raw = int(round(shadow_curvature / 1e-6))
    shadow_curvature_raw = max(-32768, min(32767, shadow_curvature_raw)) & 0xFFFF
    dat[4] |= 1 if angle_mode_engaged else 0
    dat[5] = (shadow_curvature_raw >> 8) & 0xFF
    dat[6] = shadow_curvature_raw & 0xFF
    return libsafety_py.make_CANPacket(addr, bus, bytes(dat))

  # LCA command
  def _lat_ctl_msg(self, enabled: bool, path_offset: float, path_angle: float, curvature: float, curvature_rate: float, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.cnt_lat_ctl * 50000)
      self.__class__.cnt_lat_ctl += 1
    if self.STEER_MESSAGE == MSG_LateralMotionControl:
      values = {
        "LatCtl_D_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCurv_NoRate_Actl": curvature_rate,  # Curvature rate [-0.001024|0.00102375] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl", 0, values)
    elif self.STEER_MESSAGE == MSG_LateralMotionControl2:
      values = {
        "LatCtl_D2_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCrv_NoRate2_Actl": curvature_rate,  # Curvature rate [-0.001024|0.001023] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  # Cruise control buttons
  def _acc_button_msg(self, button: int, bus: int):
    values = {
      "CcAslButtnCnclPress": 1 if button == Buttons.CANCEL else 0,
      "CcAsllButtnResPress": 1 if button == Buttons.RESUME else 0,
      "TjaButtnOnOffPress": 1 if button == Buttons.TJA_TOGGLE else 0,
    }
    return self.packer.make_can_msg_safety("Steering_Data_FD1", bus, values)

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in [True, False]:
      for msg_type in ["speed", "speed_2", "yaw"]:
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail, checksum is not checked for 2nd speed
        msg[0].data[3] = 0  # Speed checksum & half of yaw signal
        should_rx = msg_type == "speed_2" and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """Tests rx hook correctly parses the curvature measurement from the vehicle speed and yaw rate"""
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, self.MAX_CURVATURE * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._yaw_rate_msg(c, speed))

        self.assertEqual(self.safety.get_angle_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_angle_meas_max(), round(curvature * self.DEG_TO_CAN))

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_angle_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_angle_meas_min(), 0)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

  test_max_lateral_acceleration = upstream_ford.TestFordSafetyBase.test_max_lateral_acceleration

  def test_steer_allowed(self):
    # The BP four-signal path accepts small nonzero feedforward values. Test each
    # from a neutral accepted frame, including inactive and disallowed controls.
    for speed in (5., 11., 25.):
      for controls_allowed in (False, True):
        for enabled in (False, True):
          for offset, angle, curvature, rate, valid in (
            (0, 0, 0, 0, True), (0.01, 0.005, 0, 1e-6, True),
            (1.01, 0, 0, 0, False), (0, 0.251, 0, 0, False),
            (0, 0, 0.02, 0, False),
          ):
            self.setUp()
            self._reset_curvature_measurement(0, speed)
            self.safety.set_controls_allowed(controls_allowed)
            expected = (valid and controls_allowed) if enabled else (offset == angle == curvature == rate == 0)
            with self.subTest(speed=speed, controls_allowed=controls_allowed, enabled=enabled,
                              offset=offset, angle=angle, curvature=curvature, rate=rate):
              self.assertEqual(expected, self._tx(self._lat_ctl_msg(enabled, offset, angle, curvature, rate)))

  test_curvature_rate_limits = upstream_ford.TestFordSafetyBase.test_curvature_rate_limits

  def test_angle_mode_corroboration_gate(self):
    """Only corroborated angle mode may use the range beyond +/-0.25 rad."""
    for speed in (5., 15., 25.):
      for sign in (-1, 1):
        for corroborated in (False, True):
          self.setUp()
          self._reset_curvature_measurement(0, speed)
          self.safety.set_controls_allowed(True)
          self.assertTrue(self._tx(self._lka_bp_status_msg(corroborated, 0)))
          for angle in np.arange(0, 0.251, 0.005):
            self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, sign * angle, 0, 0)))
          self.assertEqual(corroborated, self._tx(self._lat_ctl_msg(True, 0, sign * 0.2505, 0, 0)))

  def test_rejected_path_command_does_not_advance_rate_history(self):
    self._reset_curvature_measurement(0, 25.)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lka_bp_status_msg(True, 0)))
    for _ in range(5):
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.1, 0, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.005, 0, 0)))

  def test_rejected_feedforward_cannot_ratchet_curvature(self):
    # Each curvature increment fits the current jerk limit, but the entire
    # message is rejected by the separate feedforward envelope. Those commands
    # never reach the EPS and must not create a new curvature rate-limit origin.
    for sign in (-1, 1):
      for lateral_only in (False, True):
        for offset, angle in ((1.01, 0), (0, 0.251)):
          with self.subTest(sign=sign, lateral_only=lateral_only, offset=offset, angle=angle):
            self.setUp()
            self._reset_curvature_measurement(0, 25.)
            self.safety.set_controls_allowed(not lateral_only)
            self.safety.set_controls_allowed_lateral(lateral_only)
            for curvature_can in (15, 30, 45, 60, 75):
              self.assertFalse(self._tx(self._lat_ctl_msg(True, offset, angle, sign * curvature_can / self.DEG_TO_CAN, 0)))
            self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, sign * 90 / self.DEG_TO_CAN, 0)))

  def test_rejected_feedforward_preserves_curvature_recovery(self):
    # A donor-only rejection must also leave a valid return toward zero
    # available from the last accepted command (15 CAN units at 25 m/s).
    for sign in (-1, 1):
      for lateral_only in (False, True):
        for offset, angle in ((1.01, 0), (0, 0.251)):
          with self.subTest(sign=sign, lateral_only=lateral_only, offset=offset, angle=angle):
            self.setUp()
            self._reset_curvature_measurement(0, 25.)
            self.safety.set_controls_allowed(not lateral_only)
            self.safety.set_controls_allowed_lateral(lateral_only)
            self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, sign * 15 / self.DEG_TO_CAN, 0)))
            self.assertFalse(self._tx(self._lat_ctl_msg(True, offset, angle, sign * 30 / self.DEG_TO_CAN, 0)))
            self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))

  def test_rejected_shadow_cap_preserves_curvature_history(self):
    # The angle shadow acceleration bound is another Ford-local check outside
    # the shared curvature checker. Its rejection cannot silently seed zero.
    for sign in (-1, 1):
      for lateral_only in (False, True):
        with self.subTest(sign=sign, lateral_only=lateral_only):
          self.setUp()
          self._reset_curvature_measurement(0, 25.)
          self.safety.set_controls_allowed(not lateral_only)
          self.safety.set_controls_allowed_lateral(lateral_only)
          self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, sign * 15 / self.DEG_TO_CAN, 0)))
          self._reset_curvature_measurement(sign * 0.008, 25.)
          self.assertTrue(self._tx(self._lka_bp_status_msg(True, sign * 0.008)))
          self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, 0, 0)))
          self.assertEqual(self.safety.get_desired_curvature_last(), sign * 15)
          self._reset_curvature_measurement(0, 25.)
          self.assertTrue(self._tx(self._lka_bp_status_msg(False, 0)))
          self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, -sign * 15 / self.DEG_TO_CAN, 0)))

  def test_rejected_feedforward_still_counts_toward_tx_rate_limit(self):
    # Roll back command history, not the shared traffic counters. Flooding
    # rejected messages must not create an allowance for another active frame.
    self._reset_curvature_measurement(0, 25.)
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(0)
    for _ in range(20):
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 1.01, 0, 1 / self.DEG_TO_CAN, 0, increment_timer=False)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 1 / self.DEG_TO_CAN, 0, increment_timer=False)))

  def test_rejected_inactive_feedforward_preserves_disengagement_reset(self):
    # The shared checker also resets when controls are disallowed, even if the
    # zero-curvature inactive field itself is valid. Do not undo that reset.
    for sign in (-1, 1):
      for lateral_only in (False, True):
        with self.subTest(sign=sign, lateral_only=lateral_only):
          self.setUp()
          self._reset_curvature_measurement(0, 25.)
          self.safety.set_controls_allowed(True)
          for curvature_can in (15, 30, 45):
            self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, sign * curvature_can / self.DEG_TO_CAN, 0)))
          self.safety.set_controls_allowed(False)
          self.safety.set_controls_allowed_lateral(False)
          self.assertFalse(self._tx(self._lat_ctl_msg(False, 1.01, 0, 0, 0)))
          self.assertEqual(self.safety.get_desired_curvature_last(), 0)
          self.safety.set_controls_allowed(not lateral_only)
          self.safety.set_controls_allowed_lateral(lateral_only)
          self.assertTrue(self._tx(self._lka_bp_status_msg(True, 0)))
          self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, 0, 0)))

  def test_angle_reengagement_while_turning(self):
    # Angle mode always sends zero wire curvature, even when measured curvature
    # is nonzero. Disengagement must not seed its command history from the turn.
    for speed in (15., 25., 35.):
      for sign in (-1, 1):
        for lateral_only in (False, True):
          with self.subTest(speed=speed, sign=sign, lateral_only=lateral_only):
            self.setUp()
            measured = sign * 1.8 / speed ** 2
            self._reset_curvature_measurement(measured, speed)
            self.safety.set_controls_allowed(False)
            self.safety.set_controls_allowed_lateral(False)
            self.assertTrue(self._tx(self._lka_bp_status_msg(True, measured)))
            self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, 0, 0)))
            self.assertTrue(self._tx(self._lat_ctl_msg(False, 0, 0, 0, 0)))

            self.safety.set_controls_allowed(not lateral_only)
            self.safety.set_controls_allowed_lateral(lateral_only)
            # No extra inactive frame after controls become allowed: the first
            # valid active frame and sustained steering must both pass.
            for _ in range(80):
              self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, 0, 0)))

  def test_angle_recovers_after_rejected_curvature_while_turning(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.setUp()
        measured = sign * 0.003
        self._reset_curvature_measurement(measured, 25.)
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._tx(self._lka_bp_status_msg(True, measured)))
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, 0, 0)))
        # An invalid command must stay blocked, without making subsequent
        # valid zero-curvature angle commands depend on measured curvature.
        self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, sign * 0.02, 0)))
        for _ in range(5):
          self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, sign * 0.005, 0, 0)))

  def test_reset_frame_cannot_bypass_curvature_limit(self):
    self._reset_curvature_measurement(0, 25.)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))
    for _ in range(65):
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0.02, 0)))

  def test_rejected_status_does_not_enable_angle_mode(self):
    self._reset_curvature_measurement(0, 15.)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._lka_bp_status_msg(True, 0, action=1)))
    for angle in np.arange(0, 0.251, 0.005):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, angle, 0, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2505, 0, 0)))

  def test_curvature_mode_unaffected_by_angle_mode_flag(self):
    """Real (nonzero) curvature commands are self-evidently curvature mode by their own content --
    they must never be gated by Lane_Assist_Data1's angle_mode_engaged, regardless of its value."""
    self.safety.set_controls_allowed(True)
    speed = 15.0
    curvature = 0.01
    self._reset_curvature_measurement(curvature, speed)
    for angle_mode_engaged in (True, False):
      self._set_prev_desired_angle(curvature)
      self._tx(self._lka_bp_status_msg(angle_mode_engaged, 0.0))
      with self.subTest(angle_mode_engaged=angle_mode_engaged):
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0)))

  def test_shadow_curvature_deviation_check(self):
    """Angle mode's shadow_curvature must be checked against measured curvature (angle_meas),
    gated the same way as curvature mode's own check: enforce_angle_error + CURVATURE_ERROR_MIN_SPEED.
    Mirrors test_curvature_rate_limits' up/down structure but for the deviation-only path.
    path_angle held at a small nonzero value -- see test_angle_mode_corroboration_gate docstring."""
    self.safety.set_controls_allowed(True)
    for speed in (self.CURVATURE_ERROR_MIN_SPEED - 1, self.CURVATURE_ERROR_MIN_SPEED + 1):
      limit_enforced = speed > self.CURVATURE_ERROR_MIN_SPEED
      measured_curvature = 0.005
      self._reset_curvature_measurement(measured_curvature, speed)

      self._tx(self._lka_bp_status_msg(True, measured_curvature))
      with self.subTest(speed=speed, case="matches_measured"):
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.01, 0, 0)))

      large_deviation = measured_curvature + (self.MAX_CURVATURE_ERROR * 5)
      self._tx(self._lka_bp_status_msg(True, large_deviation))
      with self.subTest(speed=speed, case="large_deviation"):
        self.assertEqual(not limit_enforced, self._tx(self._lat_ctl_msg(True, 0, 0.01, 0, 0)))

  def test_shadow_curvature_no_rate_limit(self):
    """shadow_curvature must NOT be rate-of-change limited -- only path_angle's own ROC
    (path_angle_cmd_checks) applies in angle mode. A large frame-to-frame jump in shadow_curvature,
    while it stays within deviation tolerance of a correspondingly-updated measured curvature, must
    not block. Regression test for a real bug: substituting shadow_curvature into
    steer_angle_cmd_checks (which does both ROC and deviation) caused spurious blocks from
    shadow_curvature's own frame-to-frame movement, unrelated to path_angle's actual behavior.
    path_angle held at a small nonzero value -- see test_angle_mode_corroboration_gate docstring."""
    self.safety.set_controls_allowed(True)
    speed = self.CURVATURE_ERROR_MIN_SPEED + 1
    for curvature in (0.015, -0.015, 0.018, -0.018, 0.001, -0.019):
      self._reset_curvature_measurement(curvature, speed)
      self._tx(self._lka_bp_status_msg(True, curvature))
      with self.subTest(curvature=curvature):
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.01, 0, 0)))

  def test_prevent_lkas_action(self):
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

  def test_acc_buttons(self):
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for enabled in (True, False):
        self._rx(self._pcm_status_msg(enabled))
        self.assertTrue(self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, 2)))

    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for bus in (0, 2):
        self.assertEqual(allowed, self._tx(self._acc_button_msg(Buttons.RESUME, bus)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      for bus in (0, 2):
        self.assertEqual(enabled, self._tx(self._acc_button_msg(Buttons.CANCEL, bus)))

  def test_enable_control_allowed_from_acc_main_on(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for main_button_msg_valid in (True, False):
          with self.subTest("main_button_msg_valid", state_valid=main_button_msg_valid):
            self.safety.set_mads_params(enable_mads, False, False)
            self._rx(self._pcm_status_msg(main_button_msg_valid))
            self.assertEqual(enable_mads and main_button_msg_valid, self.safety.get_controls_allowed_lateral())


class TestFordCANFDStockSafety(TestFordSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | FordSafetyFlagsSP.BLUEPILOT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.CANFD)
    self.safety.init_tests()


class TestFordLongitudinalSafetyBase(TestFordSafetyBase):
  MAX_ACCEL = 2.0  # accel is used for brakes, but openpilot can set positive values
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  MAX_GAS = 2.0
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  # ACC command
  def _acc_command_msg(self, gas: float, brake: float, brake_actuation: bool, cmbb_deny: bool = False):
    values = {
      "AccPrpl_A_Rq": gas,                              # [-5|5.23] m/s^2
      "AccPrpl_A_Pred": gas,                            # [-5|5.23] m/s^2
      "AccBrkTot_A_Rq": brake,                          # [-20|11.9449] m/s^2
      "AccBrkPrchg_B_Rq": 1 if brake_actuation else 0,  # Pre-charge brake request: 0=No, 1=Yes
      "AccBrkDecel_B_Rq": 1 if brake_actuation else 0,  # Deceleration request: 0=Inactive, 1=Active
      "CmbbDeny_B_Actl": 1 if cmbb_deny else 0,         # [0|1] deny AEB actuation
    }
    return self.packer.make_can_msg_safety("ACCDATA", 0, values)

  def test_stock_aeb(self):
    # Test that CmbbDeny_B_Actl is never 1, it prevents the ABS module from actuating AEB requests from ACCDATA_2
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for cmbb_deny in (True, False):
        should_tx = not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, controls_allowed, cmbb_deny)))
        should_tx = controls_allowed and not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.MAX_GAS, self.MAX_ACCEL, controls_allowed, cmbb_deny)))

  def test_gas_safety_check(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for gas in np.concatenate((np.arange(self.MIN_GAS - 2, self.MAX_GAS + 2, 0.05), [self.INACTIVE_GAS])):
        gas = round(gas, 2)  # floats might not hit exact boundary conditions without rounding
        should_tx = (controls_allowed and self.MIN_GAS <= gas <= self.MAX_GAS) or gas == self.INACTIVE_GAS
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(gas, self.INACTIVE_ACCEL, controls_allowed)))

  def test_brake_safety_check(self):
    brake_values = self._boundary_values([self.MIN_ACCEL, self.MAX_ACCEL, self.INACTIVE_ACCEL],
                                         self.MIN_ACCEL - 2, self.MAX_ACCEL + 2, 0.05)
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for brake_actuation in (True, False):
        for brake in brake_values:
          should_tx = (controls_allowed and self.MIN_ACCEL <= brake <= self.MAX_ACCEL) or brake == self.INACTIVE_ACCEL
          should_tx = should_tx and (controls_allowed or not brake_actuation)
          self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, brake, brake_actuation)))


class TestFordLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | FordSafetyFlagsSP.BLUEPILOT)
    # BP makes the alpha-long flag explicit on CAN as well as CAN-FD.
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()


class TestFordCANFDLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | FordSafetyFlagsSP.BLUEPILOT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD)
    self.safety.init_tests()


# =============================================================================
# BluePilot: steering-angle curvature measurement (FordSafetyFlagsSP.STEER_ANGLE_CURVATURE)
#
# Opt-in alternative angle_meas source for vehicles whose RCM broadcasts implausible yaw
# while its quality flag reads OK. The classes below run the ENTIRE stock test matrix with
# angle_meas sourced from SteeringPinion_Data and the widened 0.003 error band, plus
# pinion-specific tests. The stock (flag-off) classes above never set SAFETY_PARAM_SP, so
# their outcomes (including any pre-existing failures) must stay bit-identical to the base
# branch -- that comparison is the default-off zero-delta check.
# =============================================================================

class TestFordPinionCurvatureSafetyBase(TestFordSafetyBase):
  MAX_CURVATURE_ERROR = 0.003  # widened: raw pinion angle has no roll/offset compensation in firmware

  # Per-platform geometry (see the *PinionGeometry mixins). Values must match the
  # ford_pinion_geometry row for GEOMETRY_INDEX -- the table itself is checked against
  # CarSpecs + calc_slip_factor by TestFordPinionGeometryTable, so these literals only
  # need to agree with that already-verified table.
  GEOMETRY_INDEX = 0
  PINION_SLIP_FACTOR = 0.0
  PINION_STEER_RATIO = 1.0
  PINION_WHEELBASE = 1.0

  cnt_pinion = 0

  def test_curvature_rate_limits(self):
    for speed in (5., 11., 15., 25., 35.):
      self._reset_curvature_measurement(0, speed)
      delta = min(self._get_max_curvature_delta_can(speed), round(self.MAX_CURVATURE * self.DEG_TO_CAN))
      for sign in (-1, 1):
        for step, expected in ((delta, True), (delta + 1, False)):
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(0)
          self.assertEqual(expected, self._tx(self._lat_ctl_msg(True, 0, 0, sign * step / self.DEG_TO_CAN, 0)))
      # Pinion samples are rounded to their DBC precision. Check the exact band
      # around the resulting measured sample independently of the RX conversion.
      if speed > self.CURVATURE_ERROR_MIN_SPEED:
        error = round(self.MAX_CURVATURE_ERROR * self.DEG_TO_CAN)
        cap = self._get_max_curvature_can(speed)
        for sign in (-1, 1):
          for distance, expected in ((error + 1, True), (error + 2, False)):
            if distance > cap:
              continue  # tested by max-lateral-acceleration, not the error band
            curvature = sign * distance / self.DEG_TO_CAN
            self.safety.set_controls_allowed(True)
            self._set_prev_desired_angle(curvature)
            self.assertEqual(expected, self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0)))

  def _curvature_to_pinion_angle_deg(self, curvature: float, speed: float) -> float:
    # Inverse of the firmware conversion in ford_rx_hook (modes/ford.h):
    # curvature = angle_rad * curvature_factor(speed) / steer_ratio
    speed = max(speed, 0.1)
    curvature_factor = 1. / (1. - (self.PINION_SLIP_FACTOR * (speed ** 2))) / self.PINION_WHEELBASE
    angle_rad = curvature * self.PINION_STEER_RATIO / curvature_factor
    return float(np.degrees(angle_rad))

  def _pinion_quant_tol(self, speed: float) -> int:
    # 0.1 deg DBC quantization -> curvature CAN units at this speed (+2 for float rounding)
    speed = max(speed, 0.1)
    curvature_factor = 1. / (1. - (self.PINION_SLIP_FACTOR * (speed ** 2))) / self.PINION_WHEELBASE
    return int(np.radians(0.1) * curvature_factor / self.PINION_STEER_RATIO * self.DEG_TO_CAN) + 2

  # Current curvature measurement (pinion-angle sourced, not yaw)
  def _pinion_msg(self, curvature: float, speed: float, quality_flag=True):
    values = {"StePinComp_An_Est": self._curvature_to_pinion_angle_deg(curvature, speed),
              "StePinCompAnEst_D_Qf": 3 if quality_flag else 0,
              "StePinAn_No_Cnt": self.cnt_pinion % 16}
    self.__class__.cnt_pinion += 1
    return self.packer.make_can_msg_safety("SteeringPinion_Data", 0, values)

  def _reset_curvature_measurement(self, curvature, speed):
    # 14 frames, not 6: frames after a counter discontinuity (e.g. rejected bad-QF frames
    # advanced the python-side counter) are dropped by the rx counter check until it
    # re-syncs, which would otherwise leave stale samples in the 6-deep angle_meas buffer
    for _ in range(14):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))
      self._rx(self._pinion_msg(curvature, speed))

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks (stock matrix + the pinion message)
    for quality_flag in [True, False]:
      for msg_type in ["speed", "speed_2", "yaw", "pinion"]:
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)
          elif msg_type == "pinion":
            msg = self._pinion_msg(0, 0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail; checksum is not checked for 2nd speed or pinion
        # (pinion has an unknown OEM checksum algorithm; integrity is via counter + quality flag)
        msg[0].data[3] = 0  # Speed checksum & half of yaw/pinion angle signal
        should_rx = msg_type in ("speed_2", "pinion") and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """Tests rx hook correctly parses the curvature measurement from the steering pinion angle.

    The DBC signal quantizes to 0.1 deg, so allow the quantization-equivalent CAN-unit
    tolerance from the round trip through the packer.
    """
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, self.MAX_CURVATURE * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._pinion_msg(c, speed))

        quant_tol = self._pinion_quant_tol(speed)
        self.assertAlmostEqual(self.safety.get_angle_meas_min(), round(-curvature * self.DEG_TO_CAN), delta=quant_tol)
        self.assertAlmostEqual(self.safety.get_angle_meas_max(), round(curvature * self.DEG_TO_CAN), delta=quant_tol)

        self._rx(self._pinion_msg(0, speed))
        self.assertAlmostEqual(self.safety.get_angle_meas_min(), round(-curvature * self.DEG_TO_CAN), delta=quant_tol)
        self.assertAlmostEqual(self.safety.get_angle_meas_max(), 0, delta=quant_tol)

        self._rx(self._pinion_msg(0, speed))
        self.assertAlmostEqual(self.safety.get_angle_meas_min(), 0, delta=quant_tol)
        self.assertAlmostEqual(self.safety.get_angle_meas_max(), 0, delta=quant_tol)

  def test_pinion_quality_flag_gates_measurement(self):
    """A bad pinion quality flag must reject the message (measurement not updated)."""
    speed = self.CURVATURE_ERROR_MIN_SPEED + 5
    self._reset_curvature_measurement(0.005, speed)
    meas_max_before = self.safety.get_angle_meas_max()
    self.assertGreater(meas_max_before, 0)

    # bad-QF frames must be rejected at rx and leave angle_meas untouched
    for _ in range(6):
      self.assertFalse(self._rx(self._pinion_msg(0, speed, quality_flag=False)))
    self.assertEqual(self.safety.get_angle_meas_max(), meas_max_before)

  def test_pinion_sign_convention(self):
    """Command matching the measured curvature sign passes the error check; a sign-inverted
    command (the broken-yaw failure mode) violates above the gate speed."""
    speed = self.CURVATURE_ERROR_MIN_SPEED + 5
    curvature = 0.005  # well above MAX_CURVATURE_ERROR so the inverted case must violate

    for sign in (1, -1):
      with self.subTest(sign=sign):
        self._reset_curvature_measurement(sign * curvature, speed)
        self._set_prev_desired_angle(sign * curvature)
        self.safety.set_controls_allowed(True)
        # matching-sign command: allowed
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, sign * curvature, 0)))
        # inverted command (what a sign-flipped sensor would demand): blocked
        self._set_prev_desired_angle(-sign * curvature)
        self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, -sign * curvature, 0)))

  def test_pinion_check_inert_below_gate_speed(self):
    """Below CURVATURE_ERROR_MIN_SPEED the deviation check must not constrain commands."""
    self.safety.set_controls_allowed(True)
    speed = self.CURVATURE_ERROR_MIN_SPEED - 2
    self._reset_curvature_measurement(0.005, speed)
    # command far from measured, but below gate: allowed (rate limits still apply, so seed prev)
    inverted = -0.005
    self._set_prev_desired_angle(inverted)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, inverted, 0)))


class FordExplorerPinionGeometry:
  """FORD_EXPLORER_MK6 -- the on-road-validated primary platform."""
  GEOMETRY_INDEX = 5
  PINION_SLIP_FACTOR = -0.00055447339
  PINION_STEER_RATIO = 16.8
  PINION_WHEELBASE = 3.025
  SAFETY_PARAM_SP = int(FordSafetyFlagsSP.STEER_ANGLE_CURVATURE) | (GEOMETRY_INDEX << FORD_PINION_GEOMETRY_SHIFT)


class FordBroncoSportPinionGeometry:
  """FORD_BRONCO_SPORT_MK1 -- smallest wheelbase in the table."""
  GEOMETRY_INDEX = 1
  PINION_SLIP_FACTOR = -0.00062819555
  PINION_STEER_RATIO = 17.7
  PINION_WHEELBASE = 2.670
  SAFETY_PARAM_SP = int(FordSafetyFlagsSP.STEER_ANGLE_CURVATURE) | (GEOMETRY_INDEX << FORD_PINION_GEOMETRY_SHIFT)


class FordF150PinionGeometry:
  """FORD_F_150_MK14 -- largest wheelbase in the table."""
  GEOMETRY_INDEX = 8
  PINION_SLIP_FACTOR = -0.00042037149
  PINION_STEER_RATIO = 17.0
  PINION_WHEELBASE = 3.990
  SAFETY_PARAM_SP = int(FordSafetyFlagsSP.STEER_ANGLE_CURVATURE) | (GEOMETRY_INDEX << FORD_PINION_GEOMETRY_SHIFT)


class TestFordPinionLongitudinalSafety(FordExplorerPinionGeometry, TestFordPinionCurvatureSafetyBase, TestFordLongitudinalSafety):
  pass


class TestFordPinionCANFDStockSafety(FordExplorerPinionGeometry, TestFordPinionCurvatureSafetyBase, TestFordCANFDStockSafety):
  pass


class TestFordPinionCANFDLongitudinalSafety(FordExplorerPinionGeometry, TestFordPinionCurvatureSafetyBase, TestFordCANFDLongitudinalSafety):
  pass


class TestFordPinionBroncoSportSafety(FordBroncoSportPinionGeometry, TestFordPinionCurvatureSafetyBase, TestFordLongitudinalSafety):
  pass


class TestFordPinionF150Safety(FordF150PinionGeometry, TestFordPinionCurvatureSafetyBase, TestFordCANFDLongitudinalSafety):
  pass


class TestFordPinionGeometryTable(unittest.TestCase):
  """The firmware geometry table must match CarSpecs + calc_slip_factor(VehicleModel(CP))
  for every supported platform, so the table cannot rot as platforms change. Reads the
  table through the ALLOW_DEBUG libsafety getters -- no header parsing."""

  TX_MSGS: list = []  # not a CarSafetyTest; keeps common.py's cross-mode TX sweep happy

  def test_geometry_matches_carspecs(self):
    safety = libsafety_py.libsafety
    count = safety.get_ford_pinion_geometry_count()
    self.assertEqual(count, len(FORD_PINION_GEOMETRY_INDEX))
    # the index rides bits 1-4 of current_safety_param_sp; growing past 15 would silently
    # disable the firmware side while the control side still enables -- never allow it
    self.assertLessEqual(count, 15)

    seen = set()
    for car in CAR:
      if car.config.flags & FordFlags.ALT_STEER_ANGLE:
        # relative pinion angle with a learned offset -- unsupported by design
        self.assertNotIn(car, FORD_PINION_GEOMETRY_INDEX)
        continue
      self.assertIn(car, FORD_PINION_GEOMETRY_INDEX, f"{car} has no geometry-table row")
      idx = FORD_PINION_GEOMETRY_INDEX[car]
      self.assertTrue(1 <= idx <= count, f"{car}: index {idx} out of range")
      self.assertNotIn(idx, seen, f"{car}: duplicate index {idx}")
      seen.add(idx)

      specs = car.config.specs
      CP = CarParams()
      CP.mass = specs.mass
      CP.wheelbase = specs.wheelbase
      CP.steerRatio = specs.steerRatio
      CP.centerToFront = specs.wheelbase * specs.centerToFrontRatio
      CP.tireStiffnessFactor = specs.tireStiffnessFactor
      CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(
        CP.mass, CP.wheelbase, CP.centerToFront, CP.tireStiffnessFactor)
      slip_factor = calc_slip_factor(VehicleModel(CP))

      self.assertAlmostEqual(safety.get_ford_pinion_geometry_steer_ratio(idx), specs.steerRatio, places=3, msg=str(car))
      self.assertAlmostEqual(safety.get_ford_pinion_geometry_wheelbase(idx), specs.wheelbase, places=3, msg=str(car))
      self.assertAlmostEqual(safety.get_ford_pinion_geometry_slip_factor(idx), slip_factor,
                             delta=abs(slip_factor) * 1e-4, msg=str(car))

  def test_invalid_index_row_is_inert(self):
    # index 0 is the reserved invalid row: zero slip, unit ratios
    safety = libsafety_py.libsafety
    self.assertEqual(safety.get_ford_pinion_geometry_slip_factor(0), 0.0)
    self.assertEqual(safety.get_ford_pinion_geometry_steer_ratio(0), 1.0)
    self.assertEqual(safety.get_ford_pinion_geometry_wheelbase(0), 1.0)


if __name__ == "__main__":
  unittest.main()
