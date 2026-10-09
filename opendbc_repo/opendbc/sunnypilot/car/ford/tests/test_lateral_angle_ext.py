"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Unit tests for angle-mode shadow-curvature publishing (bp_kappa_cmd).
#
# The shadow value is consumed by carcontroller as the input to ford.h's angle-mode
# deviation check (Lane_Assist_Data1 bytes 5-6, judged against angle_meas). These tests
# pin the command contract: active requests publish the limited curvature used to
# derive path_angle, including during driver contact. Inactive/human-turn yield
# sends mode zero and resets the reference to measured curvature.

import math
import unittest
from dataclasses import dataclass
from unittest import mock

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.car.interfaces import scale_tire_stiffness
from opendbc.sunnypilot.car.ford import fordcan_ext, lateral_curv_ext
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt


def _explorer_cp():
  CP = structs.CarParams()
  CP.mass = 2050.
  CP.wheelbase = 3.025
  CP.steerRatio = 16.8
  CP.centerToFront = CP.wheelbase * 0.44
  CP.tireStiffnessFactor = 0.82
  CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(
    CP.mass, CP.wheelbase, CP.centerToFront, CP.tireStiffnessFactor)
  return CP


class _FakeLiveDelay:
  lateralDelay = 0.2


class _FakeSubMaster:
  def __init__(self, *args, **kwargs):
    self.updated = {s: False for s in ('modelV2', 'vehicleParameters', 'selfdriveState', 'radarState', 'lateralDelay')}
    self.valid = {s: True for s in self.updated}
    self.alive = {s: True for s in self.updated}

  def update(self, timeout=0):
    pass

  def __getitem__(self, key):
    if key == 'lateralDelay':
      return _FakeLiveDelay()
    raise KeyError(key)


class _ForcedDetector:
  def __init__(self, active):
    self.active = active

  def update(self, *_args):
    return self.active

  def reset(self):
    pass


class _FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))


class _XY:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Position:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Meta:
  laneChangeState = 0
  laneChangeDirection = 0


class _OrientationRate:
  def __init__(self, z):
    self.z = z


class _Model:
  """Minimal fake modelV2, just the fields lateral_angle_ext / lane_center_trim read."""

  def __init__(self, lane_center_y=0.0, model_y=0.0, width=3.7, lane_change_state=0):
    xs = list(range(0, 60, 2))
    half = width / 2.0
    self.laneLines = [
      _XY(xs, [lane_center_y - half - 3.7] * len(xs)),
      _XY(xs, [lane_center_y - half] * len(xs)),
      _XY(xs, [lane_center_y + half] * len(xs)),
      _XY(xs, [lane_center_y + half + 3.7] * len(xs)),
    ]
    self.laneLineProbs = [0.9, 0.9, 0.9, 0.9]
    self.laneLineStds = [0.1, 0.1, 0.1, 0.1]
    self.position = _Position(xs, [model_y] * len(xs))
    # len must match ModelConstants.T_IDXS (33) -- update_angle_strategy interps orientationRate.z
    # against T_IDXS for the variable-lookup-time predicted-curvature blend.
    self.orientationRate = _OrientationRate([0.0] * 33)
    self.meta = _Meta()
    self.meta.laneChangeState = lane_change_state


@dataclass
class _CSOut:
  vEgoRaw: float = 15.0
  vEgo: float = 15.0
  steeringPressed: bool = False
  steeringAngleDeg: float = 0.0
  yawRate: float = 0.0


class _CS:
  def __init__(self, **kwargs):
    self.out = _CSOut(**kwargs)
    self.lat_ctl_lim_stat = 0


@dataclass
class _CC:
  latActive: bool = True


@dataclass
class _Actuators:
  curvature: float = 0.0


class _Harness(LateralCurvExt, LateralAngleExt):
  """Mirrors CarController's mixin composition (see carcontroller.py)."""

  def __init__(self, CP, CP_SP=None):
    self.CP = CP  # CarControllerBase initializes this before the lateral mixins.
    with mock.patch.object(lateral_curv_ext.messaging, 'SubMaster', _FakeSubMaster):
      LateralCurvExt.__init__(self, CP, CP_SP)
    LateralAngleExt.__init__(self, CP, CP_SP)


def _pinion_harness(flag):
  """Harness with the STEER_ANGLE_CURVATURE flag set (or not) on CP_SP, detector stubbed."""
  CP = _explorer_cp()
  CP_SP = structs.CarParamsSP()
  if flag:
    CP_SP.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE
  ext = _Harness(CP, CP_SP)
  ext.human_turn_detector = _ForcedDetector(False)
  return ext, CP


class TestShadowCurvaturePublishing(unittest.TestCase):
  V_EGO = 15.0
  YAW_RATE = 0.75  # rad/s -> measured curvature = -0.75 / 15 = -0.05 (OP convention)

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=self.YAW_RATE)
    self.measured = -self.YAW_RATE / self.V_EGO

  def _update(self, lat_active=True):
    return self.ext.update_angle_strategy(_CC(latActive=lat_active), self.cs, _Actuators(curvature=0.01), self.CP)

  def test_inactive_publishes_measured(self):
    result = self._update(lat_active=False)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_human_turn_override_publishes_measured(self):
    self.ext.human_turn_detector = _ForcedDetector(True)
    result = self._update()
    self.assertTrue(self.ext.angle_human_turn_active)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_stall_pulse_publishes_measured(self):
    self.ext.stall_blip_frames_left = 3
    result = self._update()
    self.assertTrue(self.ext.angle_stall_blip_active)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.ramp_type, 0)
    self.assertEqual(self.ext.stall_blip_frames_left, 2)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_pressed_publishes_limited_command(self):
    self.cs.out.steeringPressed = True
    self._update()
    self.assertFalse(self.ext.angle_human_turn_active)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured + CarControllerParams.CURVATURE_ERROR)
    self.assertNotAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_hands_off_publishes_clipped_planner_kappa(self):
    # planner wants +0.01 while measured is -0.05: the deviation clip (active above 9 m/s)
    # bounds the shadow to measured + CURVATURE_ERROR, not measured itself -- hands-off
    # behavior is unchanged by the truthful-shadow sites.
    self._update()
    expected = self.measured + CarControllerParams.CURVATURE_ERROR
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, expected)
    self.assertNotAlmostEqual(self.ext.bp_kappa_cmd, self.measured)
    self.assertTrue(self.ext.bp_curvature_deviation_limited)


class TestStallRecovery(unittest.TestCase):
  """Retain SP recovery while narrowing its reactive trigger with PR #148.

  These are command/state checks, not proof that a pulse restores physical authority.
  """

  def _scenario(self, desired=0.014, fraction=0.2, speed=10., direction=1):
    cp = _explorer_cp()
    ext = _Harness(cp)
    # A permitted user gain keeps path_angle under the separate 0.10 rad gate.
    # Otherwise that gate can mask a broken fractional test on deep demands.
    ext.low_speed_curv_factor = 0.5
    ext.model = _Model()
    ext.model.orientationRate.z = [direction * desired * speed] * 33
    cs = _CS(vEgoRaw=speed, vEgo=speed, yawRate=-direction * desired * fraction * speed)
    actuators = _Actuators(direction * desired)
    return cp, ext, cs, actuators

  def test_curve_entry_fraction_does_not_schedule_reactive_reset(self):
    for fraction in (0.65, 0.72):
      for direction in (-1, 1):
        with self.subTest(fraction=fraction, direction=direction):
          cp, ext, cs, actuators = self._scenario(desired=0.015, fraction=fraction, direction=direction)
          for _ in range(80):
            result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
            self.assertTrue(ext.bp_curvature_deviation_limited)
            self.assertLess(abs(result.path_angle), 0.10)
            self.assertFalse(ext.angle_stall_blip_active)
            self.assertEqual(ext.stall_blip_frames_left, 0)
          self.assertEqual(ext.stall_blip_count, 0)
          self.assertEqual(ext.stall_blip_hold_s, 0.)

  def test_true_fractional_stall_retains_six_frame_recovery(self):
    for fraction in (0., 0.2, 0.649):
      for direction in (-1, 1):
        with self.subTest(fraction=fraction, direction=direction):
          cp, ext, cs, actuators = self._scenario(fraction=fraction, direction=direction)
          for _ in range(20):
            result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
            if ext.stall_blip_frames_left:
              break
          self.assertEqual(ext.stall_blip_count, 1)
          self.assertEqual(ext.stall_blip_frames_left, 6)
          self.assertGreater(result.path_angle * direction, 0.)
          for frames_left in range(5, -1, -1):
            result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
            self.assertTrue(ext.angle_stall_blip_active)
            self.assertEqual(ext.stall_blip_frames_left, frames_left)
            self.assertEqual(result.path_angle, 0.)
            self.assertEqual(result.ramp_type, 0)
            self.assertAlmostEqual(ext.bp_kappa_cmd, ext.get_current_curvature(cs))
          self.assertEqual(ext.stall_blip_cooldown_s, 2.)
          result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
          self.assertFalse(ext.angle_stall_blip_active)
          self.assertGreater(result.path_angle * direction, 0.)
          self.assertLessEqual(abs(result.path_angle), 0.055)

  def test_driver_release_preserves_healthy_curve_command(self):
    # A steering correction followed by release is not evidence of an EPS stall.
    # Exercise both turn directions and the incident's approximately 47 mph speed.
    for speed in (10., 15., 21., 25.):
      for direction in (-1, 1):
        for press_frames in (9, 11, 40):
          with self.subTest(speed=speed, direction=direction, press_frames=press_frames):
            cp, ext, cs, actuators = self._scenario(desired=0.002, fraction=1., speed=speed, direction=direction)
            cs.out.steeringPressed = True
            cs.out.steeringAngleDeg = direction * 5.0
            for _ in range(press_frames):
              before = ext.update_angle_strategy(_CC(), cs, actuators, cp)
            self.assertGreater(before.path_angle * direction, 0.)
            cs.out.steeringPressed = False
            for _ in range(20):
              result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
              self.assertFalse(ext.angle_stall_blip_active)
              self.assertFalse(ext.angle_human_turn_active)
              self.assertEqual(ext.stall_blip_frames_left, 0)
              self.assertAlmostEqual(result.path_angle, before.path_angle)
            self.assertEqual(ext.stall_blip_count, 0)

  def test_driver_release_allows_curve_exit(self):
    for direction in (-1, 1):
      with self.subTest(direction=direction):
        cp, ext, cs, actuators = self._scenario(desired=0.002, fraction=1., speed=21., direction=direction)
        cs.out.steeringPressed = True
        cs.out.steeringAngleDeg = direction * 5.0
        for _ in range(20):
          previous = ext.update_angle_strategy(_CC(), cs, actuators, cp).path_angle
        cs.out.steeringPressed = False
        for desired in (0.0015, 0.001, 0.0005, 0.):
          actuators.curvature = direction * desired
          ext.model.orientationRate.z = [direction * desired * cs.out.vEgo] * 33
          result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
          self.assertFalse(ext.angle_stall_blip_active)
          self.assertFalse(ext.angle_human_turn_active)
          self.assertLess(abs(result.path_angle), abs(previous))
          if desired:
            self.assertGreater(result.path_angle * direction, 0.)
          previous = result.path_angle
        self.assertAlmostEqual(previous, 0.)

  def test_reactive_speed_driver_and_lane_change_gates_remain(self):
    for blocked_by in ('speed', 'driver', 'lane_change'):
      with self.subTest(blocked_by=blocked_by):
        cp, ext, cs, actuators = self._scenario(speed=9. if blocked_by == 'speed' else 10.)
        cs.out.steeringPressed = blocked_by == 'driver'
        ext.model.meta.laneChangeState = 1 if blocked_by == 'lane_change' else 0
        for _ in range(80):
          ext.update_angle_strategy(_CC(), cs, actuators, cp)
          self.assertFalse(ext.angle_stall_blip_active)
        self.assertEqual(ext.stall_blip_count, 0)
        self.assertEqual(ext.stall_blip_frames_left, 0)

  def test_reactive_count_remains_bounded(self):
    cp, ext, cs, actuators = self._scenario()
    active_frames = 0
    for _ in range(500):
      ext.update_angle_strategy(_CC(), cs, actuators, cp)
      active_frames += ext.angle_stall_blip_active
    self.assertEqual(ext.stall_blip_count, 3)
    self.assertEqual(active_frames, 18)

  def test_inactive_clears_pending_recovery(self):
    cp, ext, cs, actuators = self._scenario()
    ext.stall_blip_frames_left = 6
    ext.stall_blip_hold_s = 0.4
    ext.stall_blip_count = 2
    ext.update_angle_strategy(_CC(latActive=False), cs, actuators, cp)
    self.assertFalse(ext.angle_stall_blip_active)
    self.assertEqual(ext.stall_blip_frames_left, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.)
    self.assertEqual(ext.stall_blip_count, 0)

  def test_deliberate_human_turn_still_yields_and_then_resumes(self):
    for direction in (-1, 1):
      with self.subTest(direction=direction):
        cp, ext, cs, actuators = self._scenario(direction=direction)
        cs.out.steeringPressed = True
        cs.out.steeringAngleDeg = direction * 5.0
        ext.update_angle_strategy(_CC(), cs, actuators, cp)
        cs.out.steeringAngleDeg = direction * 60.0
        for _ in range(40):
          result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
        self.assertTrue(ext.angle_human_turn_active)
        self.assertEqual(result.path_angle, 0.0)
        self.assertEqual(result.ramp_type, 0)
        self.assertAlmostEqual(ext.bp_kappa_cmd, ext.get_current_curvature(cs))
        cs.out.steeringPressed = False
        result = ext.update_angle_strategy(_CC(), cs, actuators, cp)
        self.assertFalse(ext.angle_human_turn_active)
        self.assertFalse(ext.angle_stall_blip_active)
        self.assertGreater(result.path_angle * direction, 0.0)


class TestModelFallback(unittest.TestCase):
  """Losing model data must preserve planner authority and discard stale trim."""

  V_EGO = 15.0
  PLANNER_CURVATURE = 0.001
  MODEL_CURVATURE = 0.003

  def _model(self):
    model = _Model()
    model.orientationRate.z = [self.MODEL_CURVATURE * self.V_EGO] * 33
    return model

  def _run(self, model, valid=True, alive=True, blend=0.5):
    cp = _explorer_cp()
    ext = _Harness(cp)
    ext.model = model
    ext.sm.valid['modelV2'] = valid
    ext.sm.alive['modelV2'] = alive
    ext.path_angle_blend_ratio = blend
    cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=-self.PLANNER_CURVATURE * self.V_EGO)
    for _ in range(20):
      result = ext.update_angle_strategy(_CC(), cs, _Actuators(self.PLANNER_CURVATURE), cp)
    return ext, result

  def test_missing_dead_and_invalid_models_match_planner_only(self):
    _, baseline = self._run(self._model(), blend=0.0)
    for model, valid, alive in ((None, True, True), (self._model(), False, True), (self._model(), True, False)):
      with self.subTest(model_present=model is not None, valid=valid, alive=alive):
        ext, result = self._run(model, valid, alive, blend=0.8)
        self.assertAlmostEqual(result.path_angle, baseline.path_angle)
        self.assertAlmostEqual(ext.bp_kappa_cmd, self.PLANNER_CURVATURE)

  def test_malformed_orientation_rates_match_planner_only(self):
    _, baseline = self._run(self._model(), blend=0.0)
    for rates in ([], [0.1] * 17, [0.1] * 32, [0.1] * 34, [float('nan')] * 33,
                  [float('inf')] * 33, [[0.1]] * 33):
      with self.subTest(rates_length=len(rates), first=rates[0] if rates else None):
        model = self._model()
        model.orientationRate.z = rates
        ext, result = self._run(model, blend=0.8)
        self.assertAlmostEqual(result.path_angle, baseline.path_angle)
        self.assertAlmostEqual(ext.bp_kappa_cmd, self.PLANNER_CURVATURE)

  def test_healthy_model_blends_between_publication_ticks(self):
    ext, _ = self._run(self._model(), blend=0.5)
    self.assertFalse(ext.sm.updated['modelV2'])
    self.assertAlmostEqual(ext.bp_kappa_cmd, 0.5 * (self.PLANNER_CURVATURE + self.MODEL_CURVATURE))

  def test_stale_model_cannot_keep_bias_or_lane_change_scaling(self):
    for lost_check in ('valid', 'alive'):
      with self.subTest(lost_check=lost_check):
        cp = _explorer_cp()
        ext = _Harness(cp)
        ext.model = self._model()
        ext.enable_lane_positioning_ang = True
        ext.custom_path_offset_ang = 0.3
        ext.lane_centering_strength_ang = 1.0
        cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=-self.PLANNER_CURVATURE * self.V_EGO)
        for _ in range(50):
          ext.update_angle_strategy(_CC(), cs, _Actuators(self.PLANNER_CURVATURE), cp)
        self.assertGreater(ext.lane_center_trim.correction, 0.0)
        ext.model.meta.laneChangeState = 1
        ext.model.meta.laneChangeDirection = 2
        getattr(ext.sm, lost_check)['modelV2'] = False
        ext.update_angle_strategy(_CC(), cs, _Actuators(self.PLANNER_CURVATURE), cp)
        self.assertEqual(ext.lane_center_trim.correction, 0.0)
        self.assertFalse(ext.lane_change)
        self.assertAlmostEqual(ext.bp_kappa_cmd, self.PLANNER_CURVATURE)


class TestPathAngleWireLimits(unittest.TestCase):
  def setUp(self):
    self.cp = _explorer_cp()
    self.packer = CANPacker('ford_lincoln_base_pt')
    self.can = CanBus(self.cp)

  def _packed_path_angle(self, path_angle, canfd):
    # CarController negates the strategy result for both Ford message formats.
    if canfd:
      _, data, _ = fordcan_ext.create_lat_ctl2_msg(
        self.packer, self.can, 1, 0, 1, 0., -path_angle, 0., 0., 0)
      raw = ((data[3] & 0x1f) << 6) | (data[4] >> 2)
    else:
      _, data, _ = fordcan_ext.create_lat_ctl_msg(
        self.packer, self.can, True, 0, 1, 0., -path_angle, 0., 0.)
      raw = (data[3] << 3) | (data[4] >> 5)
    return (raw - 1000) * 0.0005

  def _update(self, ext, curvature):
    # At 7 m/s these curve-entry steps fit controlsd's acceleration and jerk
    # limits. Matching model and planner curvature exercises the default blend.
    ext.model = _Model()
    ext.model.orientationRate.z = [curvature * 7.] * 33
    cs = _CS(vEgoRaw=7., vEgo=7., yawRate=-curvature * 7.)
    return ext.update_angle_strategy(_CC(), cs, _Actuators(curvature), self.cp)

  def test_curve_entry_preserves_direction_after_can_packing(self):
    for sign in (-1, 1):
      for canfd in (False, True):
        with self.subTest(sign=sign, canfd=canfd):
          ext = _Harness(self.cp)
          previous_wire_angle = 0.
          for curvature in [0.046] * 80 + [0.051] + [0.056] * 5:
            lat = self._update(ext, sign * curvature)
            wire_angle = self._packed_path_angle(lat.path_angle, canfd)
            # This caught +0.5096 internally wrapping from -0.5096 to +0.5145
            # on CAN, followed by indefinitely rejected steering commands.
            self.assertAlmostEqual(wire_angle, -lat.path_angle, delta=0.00025)
            self.assertLessEqual(abs(wire_angle - previous_wire_angle), 0.0555)
            previous_wire_angle = wire_angle

  def test_hard_saturation_tracks_each_wire_boundary(self):
    for previous, desired, expected in ((0.46, 0.07, 0.46), (-0.46, -0.07, -0.515)):
      with self.subTest(previous=previous):
        ext = _Harness(self.cp)
        ext.path_angle_last = previous
        self.assertAlmostEqual(self._update(ext, desired).path_angle, expected)

  def test_clip_uses_full_representable_range_after_negation(self):
    for previous, desired, expected in ((0.449, 0.08, 0.5), (-0.47, -0.08, -0.5235)):
      with self.subTest(previous=previous):
        ext = _Harness(self.cp)
        ext.path_angle_last = previous
        lat = self._update(ext, desired)
        self.assertAlmostEqual(lat.path_angle, expected)
        for canfd in (False, True):
          self.assertAlmostEqual(self._packed_path_angle(lat.path_angle, canfd), -expected)


class TestMeasurementSelection(unittest.TestCase):
  """get_current_curvature must select by the CP_SP STEER_ANGLE_CURVATURE flag: yaw rate
  by default (stock ford.h angle_meas family), pinion angle via the vehicle model when
  the steering-angle curvature measurement is enabled (pinion ford.h angle_meas family).
  """

  V_EGO = 15.0

  def test_default_is_yaw_rate(self):
    ext, _ = _pinion_harness(flag=False)
    cs = _CS(vEgoRaw=self.V_EGO, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertFalse(ext.bp_pinion_curvature_enabled)
    self.assertAlmostEqual(ext.get_current_curvature(cs), -0.75 / self.V_EGO)

  def test_flag_selects_pinion_vehicle_model(self):
    from opendbc.car.vehicle_model import VehicleModel
    ext, CP = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=self.V_EGO, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertTrue(ext.bp_pinion_curvature_enabled)
    expected = -VehicleModel(CP).calc_curvature(math.radians(30.0), self.V_EGO, 0.0)
    self.assertAlmostEqual(ext.get_current_curvature(cs), expected)
    self.assertNotAlmostEqual(ext.get_current_curvature(cs), -0.75 / self.V_EGO)


class TestAngleParams(unittest.TestCase):
  def setUp(self):
    self.ext = _Harness(_explorer_cp())

  def test_high_speed_dampening_preserves_platform_gain(self):
    CP = _explorer_cp()
    CP.carFingerprint = CAR.FORD_F_150_MK14
    ext = _Harness(CP)
    ext.update_angle_params(_FakeParams({"FordHighSpeedDampening_ang": b"1.12"}))
    self.assertAlmostEqual(ext.path_angle_gain_lowC_highV, 0.95)
    self.assertAlmostEqual(ext.user_dampening_factor, 1.12)

  def test_high_speed_dampening_multiplies_low_curvature_high_speed_gain(self):
    self.ext.update_angle_params(_FakeParams({"FordHighSpeedDampening_ang": b"1.12"}))
    cs = _CS(vEgoRaw=26.82, vEgo=26.82)
    self.ext.update_angle_strategy(_CC(), cs, _Actuators(), self.ext.CP)
    self.assertAlmostEqual(
      self.ext.low_gain_calc,
      self.ext.path_angle_gain_lowC_highV * self.ext.user_dampening_factor,
    )

  def test_high_speed_dampening_is_clamped(self):
    for raw_value, expected in ((b"0.10", 0.25), (b"0.50", 0.50), (b"1.50", 1.25)):
      with self.subTest(raw_value=raw_value):
        self.ext.update_angle_params(_FakeParams({"FordHighSpeedDampening_ang": raw_value}))
        self.assertAlmostEqual(self.ext.user_dampening_factor, expected)


class TestInitializeFord(unittest.TestCase):
  def test_safety_param_stays_a_plain_int(self):
    """card serializes CP_SP to capnp, which rejects enum subclasses of int -- an
    IntFlag-typed safetyParam crashed card on-device. Pin the exact type."""
    from opendbc.sunnypilot.car.interfaces import _initialize_ford
    CP = structs.CarParams()
    CP.brand = 'ford'
    CP.carFingerprint = 'FORD_EXPLORER_MK6'
    CP_SP = structs.CarParamsSP()
    _initialize_ford(CP, CP_SP, {"FordPrefSteerAngleCurvature": True})
    self.assertEqual(CP_SP.safetyParam, 0xb)  # flag | (explorer index 5 << 1)
    self.assertIs(type(CP_SP.safetyParam), int)


class TestLaneCenteringIntegration(unittest.TestCase):
  """Lane centering trim (advanced lane positioning) as wired into update_angle_strategy --
  see lane_center_trim.py for the isolated unit tests of the trim itself."""

  V_EGO = 15.0

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.ext.model = _Model()
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=0.0)

  def _update(self, lat_active=True):
    return self.ext.update_angle_strategy(_CC(latActive=lat_active), self.cs, _Actuators(curvature=0.0), self.CP)

  def test_disabled_by_default(self):
    for _ in range(50):
      self._update()
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_enabling_with_offset_produces_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(500):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_strength_param_scales_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(500):
      self._update()
    full_gain_correction = self.ext.lane_center_trim.correction

    self.ext.lane_center_trim.reset()
    self.ext.lane_centering_strength_ang = 0.5
    for _ in range(500):
      self._update()
    half_gain_correction = self.ext.lane_center_trim.correction

    self.assertAlmostEqual(half_gain_correction, full_gain_correction * 0.5, places=3)

  def test_lane_change_resets_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(200):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

    self.ext.model.meta.laneChangeState = 1  # laneChangeStarting
    self._update()
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_human_turn_resets_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(200):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

    self.ext.human_turn_detector = _ForcedDetector(True)
    self._update()
    self.assertTrue(self.ext.angle_human_turn_active)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_inactive_resets_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(200):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

    self._update(lat_active=False)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)


if __name__ == '__main__':
  unittest.main()
