"""Adversarial calibration recovery tests; no car, Params database or native hooks."""
import json
from collections import deque

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal import (
  AutoCalPipeline, LOCK_STABLE_S, NUDGE_MAX_STEP, NUDGE_PERIOD_S,
  VERIFY_FAIL_HOLD_WEIGHT, VERIFY_MIN_WEIGHT,
)
from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.tests.test_angle_autocal import (
  DT, PLATFORM_GAIN_HIGH, _MockParams, _evidenced_pipe, _frame, feed_plant,
)


def fail_trial(pipe, rec, ratio=1.3, half=0):
  # Isolate the decision seam with clean post-trial response sufficient statistics.
  pipe.est.recent[half] = [VERIFY_MIN_WEIGHT + 0.1, (VERIFY_MIN_WEIGHT + 0.1) * ratio]
  pipe._judge_verifies()
  assert pipe.verify_result[half] == "failed"


@pytest.mark.parametrize("target, expected", [(1.3, 1.05), (0.7, 0.95)])
def test_full_five_hundredths_trial_remains_available(target, expected):
  pipe = _evidenced_pipe(true_low=target, true_high=target)
  assert pipe.recommend(1., 1.) == (expected, expected)
  assert NUDGE_MAX_STEP == 0.05


@pytest.mark.parametrize("applied, target", [(1.006, 1.3), (1.004, .7), (.506, .5), (1.494, 1.5)])
def test_rounding_manual_float_cannot_exceed_trial_bound(applied, target):
  pipe = _evidenced_pipe(true_low=target, true_high=target, applied=(applied, applied))
  rec = pipe.recommend(applied, applied)
  if rec is not None:
    assert all(abs(value - applied) <= NUDGE_MAX_STEP + 1e-9 for value in rec)


@pytest.mark.parametrize("target, bad_ratio", [(1.3, 1.4), (0.8, 0.4)])
def test_failed_trial_reverses_to_pre_trial_value_promptly(target, bad_ratio):
  pipe = _evidenced_pipe(true_low=target, true_high=target)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec, ratio=bad_ratio)
  assert pipe.since_nudge_s < NUDGE_PERIOD_S
  undo = pipe.recommend(*rec)
  assert undo == (1., rec[1])
  assert abs(undo[0] - rec[0]) <= NUDGE_MAX_STEP + 1e-9
  assert pipe.step_limit_units[0] == 2


def test_overshoot_is_not_accepted_just_because_it_is_inside_old_four_percent_band():
  pipe = _evidenced_pipe(true_low=1.01, true_high=1.01)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec, ratio=1.03)
  assert pipe.recommend(*rec)[0] == 1.


def test_staging_cannot_verify_a_new_trial_with_old_samples():
  pipe = _evidenced_pipe(true_low=1.1, true_high=1.1)
  assert pipe._staged
  rec = pipe.recommend(1., 1.)
  assert not pipe._staged and not pipe._hist and not pipe.peaks.buf
  for _ in range(10):
    pipe.update(_frame(10., .004, .004, low=rec[0], high=rec[1]))
  assert pipe.est.recent_response(0)[0] == 0.


def test_unapplied_trial_cannot_collect_or_verify():
  pipe = _evidenced_pipe(true_low=1.1, true_high=1.1)
  pipe.recommend(1., 1.)
  n_before = pipe.est.n
  for _ in range(1000):
    pipe.update(_frame(10., .004, .004, low=1., high=1.))
  assert pipe.est.n == n_before
  assert pipe.verify[0] is not None and pipe.verify_result[0] == ""
  assert not pipe.locked


def test_rollback_needs_observed_values_then_discards_contradicted_fit():
  pipe = _evidenced_pipe(true_low=1.3, true_high=1.3)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec)
  undo = pipe.recommend(*rec)
  n_before = pipe.est.n
  for _ in range(100):
    pipe.update(_frame(10., .004, .004, low=rec[0], high=rec[1]))
  assert pipe.recovery[0] and pipe.est.n == n_before
  pipe.update(_frame(10., .004, .004, low=undo[0], high=undo[1]))
  assert pipe.est.n == 0 and pipe.est.solve() is None
  assert pipe.recovery == {0: None, 1: None}
  assert pipe.verify_hold == {0: VERIFY_FAIL_HOLD_WEIGHT, 1: VERIFY_FAIL_HOLD_WEIGHT}
  assert pipe.recommend(*undo) is None


def test_retry_is_smaller_and_requires_new_fit_and_response_evidence():
  pipe = _evidenced_pipe(true_low=1.3, true_high=1.3)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec)
  undo = pipe.recommend(*rec)
  pipe.update(_frame(10., .004, .004, low=undo[0], high=undo[1]))
  pipe.since_nudge_s = NUDGE_PERIOD_S
  assert pipe.recommend(*undo) is None
  feed_plant(pipe.est, 1.15, 1.15, [10, 28], applied_low=undo[0], applied_high=undo[1], n_per_speed=300)
  retry = pipe.recommend(*undo)
  assert retry[0] == 1.02
  fail_trial(pipe, retry, ratio=1.5)
  undo2 = pipe.recommend(*retry)
  assert undo2[0] == 1. and pipe.step_limit_units[0] == 1
  pipe.update(_frame(10., .004, .004, low=undo2[0], high=undo2[1]))
  pipe.since_nudge_s = NUDGE_PERIOD_S
  feed_plant(pipe.est, 1.15, 1.15, [10, 28], applied_low=undo2[0], applied_high=undo2[1], n_per_speed=300)
  assert pipe.recommend(*undo2)[0] == 1.01


def test_cannot_lock_after_five_minutes_without_post_trial_evidence():
  pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
  feed_plant(pipe.est, 1.06, 1.06, [10, 28], n_per_speed=2400)
  rec = pipe.recommend(1., 1.)
  assert rec == (1.04, 1.04)
  for _ in range(int((LOCK_STABLE_S + 1.) / DT)):
    pipe.update(_frame(20., 0., 0., low=rec[0], high=rec[1]))
    assert pipe.recommend(*rec) is None
  assert not pipe.locked and pipe.stable_s == 0.
  assert all(pipe.verify.values())


def test_recovery_and_smaller_step_survive_restart():
  pipe = _evidenced_pipe(true_low=1.3, true_high=1.3)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec)
  restored = AutoCalPipeline(PLATFORM_GAIN_HIGH)
  restored.from_dict(json.loads(json.dumps(pipe.to_dict())))
  assert restored.recommend(*rec) == (1., rec[1])
  assert restored.step_limit_units[0] == 2


def test_manual_edit_wins_over_pending_rollback():
  pipe = _evidenced_pipe(true_low=1.3, true_high=1.3)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec)
  assert pipe.recommend(1.2, rec[1]) is None
  pipe.user_edit()
  assert not any(pipe.recovery.values()) and not any(pipe.verify.values())
  assert pipe.recommend(1.2, rec[1]) is None


def test_controller_restores_old_lock_with_pending_verification_as_unfinished():
  pipe = _evidenced_pipe(true_low=1.06, true_high=1.06)
  rec = pipe.recommend(1., 1.)
  pipe.locked = True  # state the previous implementation could persist
  params = _MockParams({"FordAngleAutoCal": True,
                        "FordAngleAutoCalState": json.dumps({"v": 1, "phase": "locked", "pipe": pipe.to_dict()})})
  ctl = AutoCalController(DT)
  ctl.poll_params(params, *rec, PLATFORM_GAIN_HIGH)
  assert ctl.enabled and ctl.pipeline is not None and not ctl.pipeline.locked
  assert ctl.pipeline.verify[0] is not None


def test_controller_writes_rollback_and_does_not_resume_stale_fit():
  params = _MockParams({"FordAngleAutoCal": True, "FordAngleAutoCalState": ""})
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe(true_low=1.3, true_high=1.3)
  rec = ctl.pipeline.recommend(1., 1.)
  assert ctl._apply_nudge(rec)
  fail_trial(ctl.pipeline, rec)
  ctl.feed(_frame(10., .004, .004, low=rec[0], high=rec[1]), delay_estimated=True)
  assert params.values["FordLowSpeedFactor_ang"] == 1.
  assert json.loads(params.values["FordAngleAutoCalState"])["pipe"]["recovery"][0]
  ctl.feed(_frame(10., .004, .004, low=1., high=rec[1]), delay_estimated=True)
  assert ctl.pipeline.est.n == 0


@pytest.mark.parametrize("delay_ready", [False, True])
def test_controller_write_failure_cannot_verify_unapplied_values(delay_ready):
  class FailedWriteParams(_MockParams):
    def put(self, key, value, block=False):
      if key == "FordLowSpeedFactor_ang":
        raise OSError("synthetic disk write failure")
      super().put(key, value, block)
  params = FailedWriteParams({"FordAngleAutoCal": True, "FordAngleAutoCalState": ""})
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe(true_low=1.1, true_high=1.1)
  rec = ctl.pipeline.recommend(1., 1.)
  assert not ctl._apply_nudge(rec)
  before = ctl.pipeline.est.n
  for _ in range(1000):
    ctl.feed(_frame(10., .004, .004), delay_estimated=delay_ready)
  assert ctl.pipeline.est.n == before and ctl.pipeline.verify[0] is not None
  assert "write failed" in params.values["FordAngleAutoCalError"]


def test_fresh_baseline_required_before_trial():
  pipe = _evidenced_pipe(true_low=1.3, true_high=1.3)
  pipe.est.recent = {0: [0., 0.], 1: [0., 0.]}
  assert pipe.recommend(1., 1.) is None


def test_stale_fit_overshoot_recovers_in_closed_loop():
  # The audit counterexample: old evidence says 1.30, but the fresh plant is
  # already correct at 1.00. Before this repair it made five failed checks while
  # driving factors to 1.15/1.20; contradictory data must now bring them back.
  pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
  feed_plant(pipe.est, 1.30, 1.30, [12., 28.], kappa=.0018, n_per_speed=4800)
  factors = (1., 1.)
  history = deque([1.] * 5, maxlen=5)
  rollback_seen = False
  smaller_retry_seen = False
  for i in range(24000):
    v = 12. if (i // 400) % 2 == 0 else 28.
    history.append(factors[0] if v == 12. else factors[1])
    pipe.update(_frame(v, .0018, .0018 * history[0], low=factors[0], high=factors[1]))
    pending_rollback = dict(pipe.recovery)
    rec = pipe.recommend(*factors)
    if rec is not None:
      assert all(abs(a - b) <= .05 + 1e-9 for a, b in zip(rec, factors, strict=True))
      for h in (0, 1):
        if pending_rollback[h]:
          assert rec[h] == pending_rollback[h]["to"]
          rollback_seen = True
        elif rec[h] != factors[h] and pipe.step_limit_units[h] < 5:
          assert abs(rec[h] - factors[h]) <= .02 + 1e-9
          smaller_retry_seen = True
      factors = rec
  assert rollback_seen and smaller_retry_seen
  assert all(abs(f - 1.) <= .01 + 1e-9 for f in factors), factors
  assert pipe.locked and not any(pipe.verify.values()) and not any(pipe.recovery.values())


def test_restart_after_rollback_write_observes_recovery_before_learning():
  pipe = _evidenced_pipe(true_low=1.3, true_high=1.3)
  rec = pipe.recommend(1., 1.)
  fail_trial(pipe, rec)
  rollback = pipe.recommend(*rec)
  # Simulate the process stopping after the successful factor write but before
  # the next control frame acknowledges it. The saved recovery must be idempotent.
  saved = json.loads(json.dumps(pipe.to_dict()))
  pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
  pipe.from_dict(saved)
  assert pipe.recommend(*rollback) is None
  pipe.update(_frame(10., .004, .004, low=rollback[0], high=rollback[1]))
  assert pipe.est.n == 0 and not any(pipe.recovery.values())
  assert pipe.step_limit_units[0] == 2
