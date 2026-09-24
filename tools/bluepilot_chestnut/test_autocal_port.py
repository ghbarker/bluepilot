"""SP-specific calibration integration: preserved steering and current native APIs."""
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford import lateral_curv_ext
from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.tests.test_lateral_angle_ext import (
  _FakeSubMaster, _CS, _CC, _Actuators, _Model,
)
from opendbc.sunnypilot.car.ford.tests.test_angle_autocal import DT, PLATFORM_GAIN_HIGH, _evidenced_pipe, _frame
from openpilot.cereal import log, messaging
from openpilot.common.params import Params

ROOT = Path(__file__).resolve().parents[2]
BASE = "1e33dc1812b1b0f4f8f8af78d1603a74d49e5014"
ANGLE = "opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py"


@pytest.fixture(scope="module")
def previous_angle_class():
  # Immutable actual SP implementation, not a reimplementation of its equations.
  source = subprocess.check_output(["git", "show", f"{BASE}:{ANGLE}"], cwd=ROOT, text=True)
  ns = {"__name__": "sp_angle_before_autocal"}
  exec(compile(source, ANGLE + "@" + BASE, "exec"), ns)
  return ns["LateralAngleExt"]


def angle_harness(angle_class, cp, cp_sp):
  class Harness(lateral_curv_ext.LateralCurvExt, angle_class):
    def __init__(self):
      self.CP = cp
      with patch.object(lateral_curv_ext.messaging, "SubMaster", _FakeSubMaster):
        lateral_curv_ext.LateralCurvExt.__init__(self, cp, cp_sp)
      angle_class.__init__(self, cp, cp_sp)
  return Harness()


@pytest.mark.parametrize("platform", [CAR.FORD_MUSTANG_MACH_E_MK1, CAR.FORD_F_150_MK14, CAR.FORD_EXPLORER_MK6])
@pytest.mark.parametrize("positioning", [False, True])
def test_disabled_autocal_preserves_sp_steering_commands(previous_angle_class, platform, positioning, tmp_path):
  cp = CarInterface.get_non_essential_params(platform)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, platform)
  before = angle_harness(previous_angle_class, cp, cp_sp)
  after = angle_harness(LateralAngleExt, cp, cp_sp)
  params = Params(str(tmp_path / 'params'))
  params.put_bool("enable_lane_positioning_ang", positioning, block=True)
  params.put("custom_path_offset_ang", .15, block=True)
  params.put("FordLowSpeedFactor_ang", 1.12, block=True)
  params.put("FordHighSpeedFactor_ang", .94, block=True)
  for ext in (before, after):
    ext.update_angle_params(params)
  assert not after.autocal_enabled
  # Sequential reversals, gain bands, driver overrides, PSCM limits, inactive frames,
  # and lane changes also exercise the retained shadow/reference and unwind fixes.
  for i in range(1800):
    speed = (5., 13.11, 20., 28., 35.)[(i // 60) % 5]
    requested = .018 * math.sin(i / 35.)
    model = _Model(lane_center_y=.2, model_y=.1, lane_change_state=1 if 300 <= i % 600 < 320 else 0)
    model.meta.laneChangeDirection = 1
    model.orientationRate.z = [speed * requested * (1 + j / 100.) for j in range(33)]
    cs = _CS(vEgoRaw=speed, vEgo=speed, yawRate=-.0015 * speed * math.sin(i / 35.),
             steeringPressed=80 <= i % 200 < 100, steeringAngleDeg=60. if 80 <= i % 200 < 100 else 0.)
    cs.lat_ctl_lim_stat = 2 if 420 <= i % 600 < 440 else 0
    cc = _CC(latActive=i % 250 >= 10)
    for ext in (before, after):
      ext.model = model
    old = before.update_angle_strategy(cc, cs, _Actuators(requested), cp)
    new = after.update_angle_strategy(cc, cs, _Actuators(requested), cp)
    assert new == old, (platform, positioning, i, new, old)
    assert after.bp_kappa_cmd == before.bp_kappa_cmd
  assert params.get("FordLowSpeedFactor_ang") == 1.12
  assert params.get("FordHighSpeedFactor_ang") == .94


def test_real_lateral_delay_message_controls_calibration_admission(tmp_path):
  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, cp.carFingerprint)
  ext = angle_harness(LateralAngleExt, cp, cp_sp)
  params = Params(str(tmp_path / 'params'))
  params.put_bool("FordAngleAutoCal", True, block=True)
  ext.update_angle_params(params)
  assert ext.autocal_enabled
  ext.autocal_ctl.pipeline = _evidenced_pipe()
  # Exercise actual SubMaster envelope validity/liveness and Cap'n Proto enum names.
  sm = messaging.SubMaster(["lateralDelay"], frequency=20)
  ext.sm = sm
  msg = messaging.new_message("lateralDelay", valid=True)
  msg.lateralDelay.status = log.LateralDelay.Status.unestimated
  msg.lateralDelay.calPerc = 99
  msg.lateralDelay.lateralDelay = .2
  cs = SimpleNamespace(out=SimpleNamespace(vEgoRaw=10., steeringPressed=False, steeringTorque=0., aEgo=0.,
                       wheelSpeeds=SimpleNamespace(fl=10., fr=10., rl=10., rr=10.)))
  n = ext.autocal_ctl.pipeline.est.n
  for i in range(40):
    sm.update_msgs(10. + i * .25, [msg.as_reader()])
    ext._feed_autocal(cs, .004, .004 / 1.1)
  assert ext.autocal_ctl.pipeline.est.n == n
  assert params.get("FordLowSpeedFactor_ang", return_default=True) == 1.
  msg.lateralDelay.status = log.LateralDelay.Status.estimated
  msg.lateralDelay.calPerc = 100
  for i in range(400):
    sm.update_msgs(20. + i * DT, [msg.as_reader()] if i % 5 == 0 else [])
    ext._feed_autocal(cs, .004, .004 / 1.1)
  assert ext.autocal_ctl.pipeline.est.n > n
  sm.update_msgs(45., [])  # retain estimated payload, but publisher is stale
  assert not sm.all_checks(["lateralDelay"])
  ext._feed_autocal(None, .004, .004)
  assert not ext.autocal_ctl.pipeline._hist
  assert ext.autocal_ctl.pipeline.est.recent_response(0)[0] == 0.


def test_current_params_float_writes_and_serialized_telemetry(tmp_path):
  from bluepilot.selfdrive.car.bp_card_publisher import publish_controller_state_bp
  params = Params(str(tmp_path / 'params'))
  assert not params.get_bool("FordAngleAutoCal")
  params.put_bool("FordAngleAutoCal", True, block=True)
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe()
  ctl.feed(_frame(10., .004, .004 / 1.1), delay_estimated=True)
  assert params.get("FordLowSpeedFactor_ang") == 1.05
  assert params.get("FordHighSpeedFactor_ang") == 1.05
  assert params.get("FordAngleAutoCalError", return_default=True) == ""
  ctl.poll_params(params, 1.05, 1.05, PLATFORM_GAIN_HIGH)
  cc = SimpleNamespace(lateralUncertainty=0., autocal_enabled=ctl.enabled,
                       bp_autocal_status=ctl.status, bp_angle_saturated=True)
  messages = {}
  class Publisher:
    def send(self, service, event):
      messages[service] = event.to_bytes()
  with patch('bluepilot.selfdrive.car.bp_card_publisher.Params', return_value=params):
    publish_controller_state_bp(SimpleNamespace(CC=cc, CP=None), Publisher())
  with log.Event.from_bytes(messages["controllerStateBP"]) as event:
    assert event.controllerStateBP.bmsAngleAutoCalibrate
    assert event.controllerStateBP.bmsAngleAutoCalState == ctl.status
    assert event.controllerStateBP.angleSaturated


def test_ui_clear_cannot_be_repopulated_before_slow_parameter_poll(tmp_path):
  params = Params(str(tmp_path / 'params'))
  params.put_bool("FordAngleAutoCal", True, block=True)
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe()
  ctl._save("collecting", (1., 1.))
  ctl._dirty = True
  ctl._save_s = 30.
  # Same parameter operations as the UI, before the controller's next 1 Hz poll.
  params.put("FordAngleAutoCalState", "", block=True)
  params.put_bool("FordAngleAutoCal", False, block=True)
  ctl.feed(_frame(10., .004, .004), delay_estimated=True)
  assert not ctl.enabled and ctl.pipeline is None
  assert params.get("FordAngleAutoCalState", return_default=True) == ""
  assert params.get("FordLowSpeedFactor_ang", return_default=True) == 1.


def test_remote_disable_clears_finished_lock_without_a_ui_callback(tmp_path):
  params = Params(str(tmp_path / 'params'))
  params.put_bool("FordAngleAutoCal", True, block=True)
  params.put("FordAngleAutoCalState", 'done low=1.10 high=1.05 verified', block=True)
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1.1, 1.05, PLATFORM_GAIN_HIGH)
  assert ctl.done and not ctl.enabled
  params.put_bool("FordAngleAutoCal", False, block=True)
  ctl.poll_params(params, 1.1, 1.05, PLATFORM_GAIN_HIGH)
  params.put_bool("FordAngleAutoCal", True, block=True)
  ctl.poll_params(params, 1.1, 1.05, PLATFORM_GAIN_HIGH)
  assert ctl.enabled and not ctl.done and ctl.pipeline.est.n == 0
  assert ctl._last_written == (1.1, 1.05)
