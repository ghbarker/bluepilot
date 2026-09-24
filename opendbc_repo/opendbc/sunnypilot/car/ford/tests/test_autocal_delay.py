"""Delay-learning interlock: real controller/pipeline and isolated caller method.

The method fixture executes the production _feed_autocal body without importing
the native car stack. It checks that seam, not native messaging or vehicle behavior.
"""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal import Frame, LOCK_STABLE_S
from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.tests.test_angle_autocal import (
  DT, PLATFORM_GAIN_HIGH, _MockParams, _evidenced_pipe, _frame, feed_plant,
)
from opendbc.sunnypilot.car.ford.tests.test_autocal_recovery import fail_trial


def controller(pipe=None):
  params = _MockParams({"FordAngleAutoCal": True, "FordAngleAutoCalState": ""})
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  if pipe is not None:
    ctl.pipeline = pipe
  return ctl, params


@pytest.fixture
def feed_method():
  path = Path(__file__).parents[1] / "lateral_angle_ext.py"
  tree = ast.parse(path.read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "LateralAngleExt")
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "_feed_autocal")
  ns = {"Frame": Frame}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), ns)
  return ns[method.name]


class DelayMessages:
  def __init__(self, status="estimated", progress=100, valid=True, alive=True, freq_ok=True, delay=.2):
    self.delay = SimpleNamespace(status=status, calPerc=progress, lateralDelay=delay)
    self.checks = (valid, alive, freq_ok)
    self.updated = {"lateralDelay": False}  # 4 Hz publisher, 20 Hz consumer: normal between messages

  def __getitem__(self, service):
    assert service == "lateralDelay"
    return self.delay

  def all_checks(self, services):
    assert services == ["lateralDelay"]
    return all(self.checks)


@pytest.mark.parametrize("state", [
  {"status": "unestimated", "progress": 0},
  {"status": "unestimated", "progress": 99},
  {"status": "invalid"},
  {"progress": 99},
  {"valid": False},
  {"alive": False},
  {"freq_ok": False},
])
def test_caller_blocks_learning_and_unhealthy_estimated_payloads(feed_method, state):
  ctl, params = controller(_evidenced_pipe())
  ext = SimpleNamespace(autocal_ctl=ctl, sm=DelayMessages(**state))
  n = ctl.pipeline.est.n
  # No vehicle frame should even be read when delay is unready.
  feed_method(ext, None, .004, .004)
  assert ctl.pipeline.est.n == n and not params.written
  assert not ctl.pipeline._hist and not ctl.pipeline._staged
  assert all(ctl.pipeline.est.recent_response(h)[0] == 0 for h in (0, 1))


def test_caller_accepts_healthy_delay_between_publisher_updates(feed_method):
  ctl, _ = controller()
  ext = SimpleNamespace(autocal_ctl=ctl, sm=DelayMessages(), bp_angle_rate_limited=False,
                        bp_curvature_deviation_limited=False, bp_angle_saturated=False,
                        low_speed_curv_factor=1., high_speed_curv_factor=1.)
  cs = SimpleNamespace(out=SimpleNamespace(wheelSpeeds=SimpleNamespace(fl=10., fr=10., rl=10., rr=10.),
                       vEgoRaw=10., steeringPressed=False, steeringTorque=0., aEgo=0.))
  for _ in range(200):
    feed_method(ext, cs, .004, .004)
  assert ctl.pipeline.est.n > 0


def test_disabled_caller_does_not_read_delay_or_vehicle_signals(feed_method):
  ctl = AutoCalController(DT)
  feed_method(SimpleNamespace(autocal_ctl=ctl), None, 0., 0.)
  assert ctl.pipeline is None


@pytest.mark.parametrize("delay", [float("nan"), float("inf"), -float("inf"), 0., -.2])
def test_unusable_delay_cannot_reach_alignment_or_parameter_writes(delay):
  ctl, params = controller(_evidenced_pipe())
  n = ctl.pipeline.est.n
  ctl.feed(_frame(10., .004, .004, lat_delay=delay), delay_estimated=True)
  assert ctl.pipeline.est.n == n and not params.written
  assert ctl.pipeline.stable_s == 0.
  assert not ctl.pipeline._hist and not ctl.pipeline._staged


@pytest.mark.parametrize("restored_evidence", [False, True])
def test_learning_blocks_samples_nudges_and_completion_for_entire_wait(restored_evidence):
  ctl, params = controller(_evidenced_pipe() if restored_evidence else None)
  pipe = ctl.pipeline
  n, fit = pipe.est.n, pipe.est.solve()
  pipe.stable_s = LOCK_STABLE_S - DT
  for _ in range(int((LOCK_STABLE_S + 1.) / DT)):
    ctl.feed(_frame(10., .004, .004), delay_estimated=False)
  assert pipe.est.n == n and pipe.est.solve() == fit
  assert pipe.since_nudge_s == 0. and pipe.stable_s == 0.
  assert not pipe.locked and not params.written
  assert not pipe.peaks.buf and not pipe._hist and not pipe._staged


def test_ready_learning_ready_requires_fresh_curves_before_next_trial():
  ctl, params = controller(_evidenced_pipe())
  # Already enough evidence to adjust, then delay returns to learning.
  ctl.feed(_frame(10., .004, .004 / 1.1), delay_estimated=False)
  n = ctl.pipeline.est.n
  ctl.feed(_frame(10., .004, .004 / 1.1), delay_estimated=True)
  assert not params.written and ctl.pipeline.est.n == n
  assert ctl.pipeline.est.recent_response(0)[0] == 0.
  for _ in range(int(25. / DT)):
    ctl.feed(_frame(10., .004, .004 / 1.1), delay_estimated=True)
    if "FordLowSpeedFactor_ang" in params.written:
      break
  assert params.written["FordLowSpeedFactor_ang"] == 1.05
  assert params.written["FordHighSpeedFactor_ang"] == 1.  # no fresh high-speed evidence


def test_pre_pause_responses_cannot_verify_pending_trial():
  ctl, params = controller(_evidenced_pipe())
  pipe = ctl.pipeline
  rec = pipe.recommend(1., 1.)
  pending = dict(pipe.verify[0])
  pipe.est.recent[0] = [10., 10.]  # enough to confirm if reused
  ctl.feed(_frame(10., .004, .004, low=rec[0], high=rec[1]), delay_estimated=False)
  ctl.feed(_frame(10., .004, .004, low=rec[0], high=rec[1]), delay_estimated=True)
  assert pipe.verify[0] == pending and pipe.verify_result[0] == ""
  assert pipe.est.recent_response(0)[0] == 0. and not params.written


def test_pending_rollback_is_frozen_while_delay_learns():
  ctl, params = controller(_evidenced_pipe())
  pipe = ctl.pipeline
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec)
  for _ in range(100):
    ctl.feed(_frame(10., .004, .004, low=rec[0], high=rec[1]), delay_estimated=False)
  assert not params.written and pipe.recovery[0] is not None
  assert pipe.step_limit_units[0] == 2
  ctl.feed(_frame(10., .004, .004, low=rec[0], high=rec[1]), delay_estimated=True)
  assert params.written["FordLowSpeedFactor_ang"] == 1.


def test_stale_near_complete_fit_cannot_lock_on_straights_after_relearning():
  ctl, _ = controller()
  pipe = ctl.pipeline
  feed_plant(pipe.est, 1., 1., [10, 28], n_per_speed=2400)
  pipe.stable_s = LOCK_STABLE_S - DT
  ctl.feed(_frame(20., 0., 0.), delay_estimated=False)
  for _ in range(int((LOCK_STABLE_S + 1.) / DT)):
    ctl.feed(_frame(20., 0., 0.), delay_estimated=True)
  assert ctl.pipeline is pipe and not pipe.locked and pipe.stable_s == 0.


def test_restart_cannot_nudge_from_persisted_recent_responses_even_if_delay_ready():
  donor = _evidenced_pipe()
  params = _MockParams({"FordAngleAutoCal": True,
                        "FordAngleAutoCalState": json.dumps({"v": 1, "phase": "collecting", "pipe": donor.to_dict()})})
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  assert ctl.pipeline.est.n == donor.est.n and ctl.pipeline.est.solve() == donor.est.solve()
  ctl.feed(_frame(10., .004, .004), delay_estimated=True)
  assert not params.written and ctl.pipeline.est.recent_response(0)[0] == 0.
