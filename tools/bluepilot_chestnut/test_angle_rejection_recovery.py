"""Exercise real Ford interface/controller recovery against unchanged Panda hooks."""
from unittest.mock import patch

import pytest

from opendbc.car import Bus, structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket
from opendbc.sunnypilot.car.ford.steering_diagnostics import steering_command_snapshot
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP
from openpilot.common.params import Params
from tools.bluepilot_chestnut.test_angle_command_consistency import MachEPinionSafety


def run_controller(tmp_path, sign, speed, reject_frame=None, provide_returns=True, deactivate_frame=None, stock_frame=None):
  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, cp.carFingerprint)
  cp_sp.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (11 << 1)
  params = Params(str(tmp_path / 'params'))
  params.put('FordPrefLateralControl', 1, block=True)
  with patch('opendbc.car.ford.carcontroller.Params', return_value=params):
    ci = CarInterface(cp, cp_sp)
  ci.CC.angle_command_recovery.clock = lambda: now
  cc, cc_sp = structs.CarControl(), structs.CarControlSP()
  cc.latActive = True
  cc.actuators.curvature = sign * .003
  cs = structs.CarState()
  cs.vEgoRaw = cs.vEgo = speed
  cs.canValid = True
  ci.CS.buttons_stock_values = {}
  safety = MachEPinionSafety()
  safety.setUp()
  safety.safety.set_controls_allowed(True)
  safety._reset_curvature_measurement(0., speed)
  pending = []
  rows = []
  for frame in range(41):
    now = 10_000_000_000 + frame * 10_000_000
    # Pass Panda responses through the real Ford interface. Unrelated input
    # parsers have no synthetic vehicle bus here, so restore fixed vehicle data.
    ci.update([(now, pending if provide_returns else [])])
    pending = []
    ci.CS.out = cs
    ci.CS.acc_tja_status_stock_values = ci.can_parsers[Bus.cam].vl['ACCDATA_3']
    ci.CS.lkas_status_stock_values = ci.can_parsers[Bus.cam].vl['IPMA_Data']
    if frame == deactivate_frame:
      cc.latActive = False
    if frame == stock_frame:
      params.put_bool('disable_BP_lat_UI', True, block=True)
    _, messages = ci.apply(cc.as_reader(), cc_sp, now)
    for message in messages:
      address, data, bus = message
      if address != 0x3D6:
        continue
      snapshot = steering_command_snapshot(message, frame, now)
      # Force one independent safety failure; a fresh valid status returns on
      # subsequent frames. Recovery must not turn this blocked frame into a TX.
      shadow = .1 if frame == reject_frame else -ci.CC.bp_kappa_cmd
      assert safety._tx(safety._lka_bp_status_msg(True, shadow))
      accepted = safety._tx(make_CANPacket(address, bus, data))
      pending.append((address, data, bus + (128 if accepted else 192)))
      rows.append((frame, snapshot['pathAngle'], snapshot['mode'], bool(accepted)))
  return rows


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('speed', [25., 35.])
def test_rejection_does_not_wind_up_host_rate_history(tmp_path, sign, speed):
  baseline = run_controller(tmp_path / 'before', sign, speed, reject_frame=10, provide_returns=False)
  recovered = run_controller(tmp_path / 'after', sign, speed, reject_frame=10)
  assert baseline[:3] == recovered[:3]
  assert recovered[2][3] is False  # The deliberate safety violation stays blocked.
  assert baseline[3][3] is False  # Old host history causes another rate rejection.
  assert all(row[3] for row in recovered[3:])
  assert all(row[2] == 1 for row in recovered[1:])  # No synthetic mode-zero pulse.
  assert abs(recovered[3][1] - recovered[1][1]) <= .00901
  assert abs(baseline[3][1] - baseline[1][1]) > .0095


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('speed', [15., 25., 35.])
def test_normal_commands_are_identical_with_or_without_returns(tmp_path, sign, speed):
  with_returns = run_controller(tmp_path / 'returns', sign, speed)
  without_returns = run_controller(tmp_path / 'no_returns', sign, speed, provide_returns=False)
  assert with_returns == without_returns
  assert all(row[3] for row in with_returns)


@pytest.mark.parametrize('transition', ['deactivate_frame', 'stock_frame'])
def test_transition_after_rejection_keeps_neutral_command(tmp_path, transition):
  rows = run_controller(tmp_path, 1, 25., reject_frame=10, **{transition: 15})
  neutral = next(row for row in rows if row[0] == 15)
  assert neutral[1:3] == (0., 0)
