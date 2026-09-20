"""Command-side regression for the Mach-E pinion/reference disagreement.

Safety firmware is deliberately unmodified. These checks use its compiled hooks;
the controller must alter the actual path request when its intent is out of range.
"""
import math
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car.ford.interface import CarInterface
from opendbc.car import structs, Bus
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests import test_ford_bluepilot as ford
from opendbc.sunnypilot.car.ford.tests.test_lateral_angle_ext import _Harness, _CS, _CC, _Actuators
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP, FORD_PINION_GEOMETRY_INDEX
from openpilot.common.params import Params


class MachEPinionSafety(ford.FordExplorerPinionGeometry, ford.TestFordPinionCurvatureSafetyBase, ford.TestFordCANFDStockSafety):
  __test__ = False  # Fixture only; the complete inherited safety suite runs separately.
  GEOMETRY_INDEX = 11
  PINION_SLIP_FACTOR = -0.00056209187
  PINION_STEER_RATIO = 17.0
  PINION_WHEELBASE = 2.984
  SAFETY_PARAM_SP = FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (GEOMETRY_INDEX << 1)


def harness(sign=1):
  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, cp.carFingerprint)
  cp_sp.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (11 << 1)
  ext = _Harness(cp, cp_sp)
  ext.lp = SimpleNamespace(angleOffsetDeg=sign * 2.54, roll=sign * 0.0799)
  ext.VM.update_params(1.00011, 16.70886)
  ext.path_angle_blend_ratio = 0.
  cs = _CS(vEgoRaw=13.11, vEgo=13.11, steeringAngleDeg=-sign * 33.2)
  return ext, cp, cs


def command(ext, cp, cs, requested):
  # Reach the requested angle through the existing rate limiter.
  for _ in range(12):
    result = ext.update_angle_strategy(_CC(), cs, _Actuators(requested), cp)
  return result


def accepts(ext, cs, result):
  safety = MachEPinionSafety()
  safety.setUp()
  safety.safety.set_controls_allowed(True)
  raw = ext.get_safety_curvature(cs)
  safety._reset_curvature_measurement(-raw, cs.out.vEgoRaw)
  assert safety._tx(safety._lka_bp_status_msg(True, -raw))
  steps = math.ceil(abs(result.path_angle) / 0.01) + 1
  for step in range(steps + 1):
    assert safety._tx(safety._lat_ctl_msg(True, 0., -result.path_angle * step / steps, 0., 0.))
  assert safety._tx(safety._lka_bp_status_msg(True, -ext.bp_kappa_cmd))
  return safety._tx(safety._lat_ctl_msg(True, 0., -result.path_angle, 0., 0.))


@pytest.mark.parametrize('sign', [-1, 1])
def test_recorded_calibration_disagreement_changes_actual_command(sign):
  old, cp, cs = harness(sign)
  # Make the new intersection inert to reproduce the previous learned-only clip.
  with patch.object(old, 'get_safety_curvature', old.get_current_curvature):
    previous = command(old, cp, cs, sign * 0.03)
  assert not accepts(old, cs, previous)

  fixed, cp, cs = harness(sign)
  result = command(fixed, cp, cs, sign * 0.03)
  assert accepts(fixed, cs, result)
  assert result.path_angle != previous.path_angle
  assert fixed.bp_kappa_cmd != old.bp_kappa_cmd
  assert fixed.bp_curvature_deviation_limited
  # The path is calculated from the guarded intent; this cannot be a shadow-only fix.
  assert result.path_angle == pytest.approx(fixed.bp_kappa_cmd * cs.out.vEgoRaw * fixed.curvature_factor)


@pytest.mark.parametrize('requested', [-0.01, 0., 0.01, 0.012])
def test_requests_already_in_both_bands_are_unchanged(requested):
  old, cp, cs = harness()
  fixed, _, _ = harness()
  with patch.object(old, 'get_safety_curvature', old.get_current_curvature):
    previous = command(old, cp, cs, requested)
  result = command(fixed, cp, cs, requested)
  assert result == previous
  assert fixed.bp_kappa_cmd == old.bp_kappa_cmd


def test_safety_reference_does_not_follow_learned_parameters():
  ext, cp, cs = harness()
  baseline = ext.get_safety_curvature(cs)
  expected = -VehicleModel(cp).calc_curvature(math.radians(cs.out.steeringAngleDeg), cs.out.vEgoRaw, 0.)
  assert baseline == pytest.approx(expected)
  assert abs(baseline - ext.get_current_curvature(cs)) > 0.001
  ext.VM.update_params(1.2, 19.)
  ext.lp.angleOffsetDeg = 5.
  ext.lp.roll = -0.1
  assert ext.get_safety_curvature(cs) == baseline


def test_conflicting_references_do_not_bypass_the_board():
  ext, cp, cs = harness()
  ext.lp.angleOffsetDeg = 40.
  result = command(ext, cp, cs, 0.04)
  assert not accepts(ext, cs, result)
  assert abs(ext.bp_kappa_cmd - ext.get_current_curvature(cs)) <= 0.002000001


@pytest.mark.parametrize('platform', list(FORD_PINION_GEOMETRY_INDEX))
def test_fixed_reference_matches_firmware_geometry(platform):
  cp = CarInterface.get_non_essential_params(platform)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, platform)
  index = FORD_PINION_GEOMETRY_INDEX[platform]
  cp_sp.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (index << 1)
  ext = _Harness(cp, cp_sp)
  safety = ford.libsafety_py.libsafety
  slip = safety.get_ford_pinion_geometry_slip_factor(index)
  ratio = safety.get_ford_pinion_geometry_steer_ratio(index)
  wheelbase = safety.get_ford_pinion_geometry_wheelbase(index)
  for speed in (0., 9., 10., 13.11, 25., 40.):
    for angle in (-180., -33.2, 0., 33.2, 180.):
      cs = _CS(vEgoRaw=speed, steeringAngleDeg=angle)
      expected = -math.radians(angle) / ((1 - slip * max(speed, 0.1)**2) * wheelbase * ratio)
      assert ext.get_safety_curvature(cs) == pytest.approx(expected, abs=1e-8)


@pytest.mark.parametrize('start_phase', range(15))
def test_controller_status_precedes_coincident_requests_without_extra_messages(start_phase, tmp_path):
  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, cp.carFingerprint)
  cp_sp.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (11 << 1)
  params = Params(str(tmp_path / 'params'))
  params.put('FordPrefLateralControl', 1, block=True)
  with patch('opendbc.car.ford.carcontroller.Params', return_value=params):
    ci = CarInterface(cp, cp_sp)
  ci.CC.frame = start_phase
  ci.CS.out = structs.CarState()
  ci.CS.out.vEgoRaw = ci.CS.out.vEgo = 13.11
  ci.CS.buttons_stock_values = {}
  ci.CS.acc_tja_status_stock_values = ci.can_parsers[Bus.cam].vl['ACCDATA_3']
  ci.CS.lkas_status_stock_values = ci.can_parsers[Bus.cam].vl['IPMA_Data']
  cc, cc_sp = structs.CarControl(), structs.CarControlSP()
  cc.latActive = True
  cc.actuators.curvature = 0.001
  counts = {0x3ca: 0, 0x3d6: 0}
  for tick in range(start_phase, start_phase + 150):
    _, messages = ci.apply(cc.as_reader(), cc_sp, tick * 10_000_000)
    addresses = [m[0] for m in messages]
    for address in counts:
      counts[address] += addresses.count(address)
    assert addresses.count(0x3ca) == (tick % CarControllerParams.LKA_STEP == 0)
    assert addresses.count(0x3d6) == (tick % CarControllerParams.STEER_STEP == 0)
    if 0x3ca in addresses and 0x3d6 in addresses:
      assert addresses.index(0x3ca) < addresses.index(0x3d6)
  assert counts == {0x3ca: 50, 0x3d6: 30}


def test_stale_or_unsafe_shadow_remains_rejected():
  ext, cp, cs = harness()
  result = command(ext, cp, cs, 0.01)
  assert accepts(ext, cs, result)
  # Simulate a status frame left over from a previous, much tighter turn.
  ext.bp_kappa_cmd = 0.02
  assert not accepts(ext, cs, result)


def test_fresh_status_cannot_hide_an_acceleration_limit_violation():
  safety = MachEPinionSafety()
  safety.setUp()
  safety.safety.set_controls_allowed(True)
  safety._reset_curvature_measurement(0.0073, 23.)
  # The new value is within the measurement-deviation band but outside the
  # separate acceleration cap. Publishing it earlier must retain that rejection.
  assert safety._tx(safety._lka_bp_status_msg(True, 0.0073))
  assert safety._tx(safety._lat_ctl_msg(True, 0., 0.01, 0., 0.))
  assert safety._tx(safety._lka_bp_status_msg(True, 0.0081))
  assert not safety._tx(safety._lat_ctl_msg(True, 0., 0.01, 0., 0.))
