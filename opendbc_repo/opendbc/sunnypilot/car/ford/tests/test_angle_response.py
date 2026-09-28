"""Adversarial qualified-evidence and trial lifecycle regressions.

Synthetic observations establish algorithm behavior, not vehicle validation.
"""
import json
import math
from dataclasses import replace
from unittest.mock import patch

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal import AutoCalPipeline, NUDGE_MAX_STEP, VERIFY_FAIL_HOLD_WEIGHT
from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.angle_response import ResponseWindow, RESPONSE_MAX_AGE_S, RESPONSE_MAX_WEIGHT
from opendbc.sunnypilot.car.ford.tests.test_angle_autocal import (
  DT, PLATFORM_GAIN_HIGH, _MockParams, _evidenced_pipe, _feed_low, _frame, applied_gain,
)


def test_sparse_qualified_turns_accumulate_without_ema_starvation():
  pipe = _evidenced_pipe()
  pipe.pause_for_delay()
  # Already-qualified observations at 0.05 weighted seconds per elapsed second.
  # The old 90s EMA asymptotes below the unchanged 6/12 evidence requirements.
  for _ in range(250):
    pipe.est.decay(1.)
    for v in (10., 28.):
      pipe.est.add_sample(v, .002, .002 / 1.1, applied_gain(v, 1., 1.), weight=.05)
  assert pipe.est.recent_response(0)[0] < 6.
  assert pipe.est.responses.response(0)[0] >= VERIFY_FAIL_HOLD_WEIGHT
  _feed_low(pipe, (1., 1.), 22., 1.1, 1.1)
  assert pipe.recommend(1., 1.) == (1.05, 1.)


def test_intermittent_turns_progress_through_real_admission_gates():
  pipe = _evidenced_pipe()
  pipe.pause_for_delay()
  # Six-second turns separated by 54-second straights, alternating speed ranges
  # and direction. No direct evidence injection after delay becomes ready.
  for i in range(int(540. / DT)):
    seconds = i * DT
    block = int(seconds // 60)
    v = 10. if block % 2 == 0 else 28.
    cmd = (.002 if (block // 2) % 2 else -.002) if seconds % 60 < 6 else 0.
    pipe.update(_frame(v, cmd, cmd / 1.1))
    trial = pipe.recommend(1., 1.)
    if trial is not None:
      assert trial == (1.05, 1.)
      assert seconds > 120.  # must accumulate evidence over multiple turns
      return
  pytest.fail('qualified intermittent turns did not clear the unchanged evidence threshold')


def test_window_caps_storage_and_expires_in_idle_time():
  window = ResponseWindow()
  for _ in range(1000):
    window.add(0, 0, .05, .9)
  assert window.response(0)[0] == pytest.approx(RESPONSE_MAX_WEIGHT)
  assert len(window.rows[0]) <= 481
  window.advance(RESPONSE_MAX_AGE_S - 1)
  assert window.response(0)[0] > 0
  window.advance(1.)
  assert window.response(0) == (0., None, math.inf)
  # Tiny interpolation weights must not leave dangling bin references.
  for _ in range(100):
    window.add(1, 7, 1e-10, .9)
  window.advance(RESPONSE_MAX_AGE_S)
  assert not window.rows[1]


def test_matching_mix_prevents_false_success_from_different_turns():
  window = ResponseWindow()
  window.add(0, 0, 18., .8)
  window.add(0, 1, 6., .98)
  baseline = window.snapshot(0)
  pre_mean = window.response(0)[1]
  window.clear()
  window.add(0, 0, 6., .8)
  window.add(0, 1, 18., .98)
  assert window.response(0)[1] > pre_mean  # naive aggregate appears to improve
  weight, before, after, _, _ = window.comparable(0, baseline)
  assert weight == 12.
  assert before == pytest.approx(after)  # unchanged response is not improvement
  window.clear()
  window.add(0, 4, 24., 1.)  # different speed/direction bin cannot confirm
  assert window.comparable(0, baseline)[0] == 0.


@pytest.mark.parametrize('v,expected', [(10., (1.05, 1.)), (28., (1., 1.05))])
def test_trial_starts_only_in_current_speed_range(v, expected):
  pipe = _evidenced_pipe()
  _feed_low(pipe, (1., 1.), 3., 1.1, 1.1, v=v, kappa=.002)
  assert pipe.recommend(1., 1.) == expected
  assert sum(p is not None for p in pipe.verify.values()) == 1


def test_unmoved_off_grid_factor_is_preserved_exactly():
  pipe = _evidenced_pipe()
  pipe._active_half = 0
  assert pipe.recommend(1., 1.003) == (1.05, 1.003)


@pytest.mark.parametrize('flag', ['steering_pressed', 'angle_rate_limited', 'deviation_limited', 'saturated'])
def test_existing_evidence_cannot_start_trial_during_intervention(flag):
  pipe = _evidenced_pipe()
  frame = replace(_frame(10., .004, .004 / 1.1), **{flag: True})
  pipe.update(frame)
  assert pipe.recommend(1., 1.) is None


@pytest.mark.parametrize('elapsed', [RESPONSE_MAX_AGE_S, math.nan, -1.])
def test_expired_trial_rolls_back_within_bound(elapsed):
  pipe = _evidenced_pipe()
  pipe._active_half = 0
  trial = pipe.recommend(1., 1.)
  cap = dict(pipe.step_limit_units)
  pipe.idle(elapsed)
  assert pipe.recommend(*trial) == (1., 1.)
  assert pipe.verify_result[0] == 'expired'
  assert abs(trial[0] - 1.) <= NUDGE_MAX_STEP + 1e-9
  assert pipe.step_limit_units == cap  # timeout is not evidence of a bad direction
  assert pipe.recommend(1.12, 1.) is None  # never overwrite a manual edit


def test_idle_gap_expires_baseline_without_restoring_freshness():
  pipe = _evidenced_pipe()
  pipe.idle(RESPONSE_MAX_AGE_S)
  _feed_low(pipe, (1., 1.), 3., 1.1, 1.1)
  assert pipe.recommend(1., 1.) is None
  assert pipe.ui_state(1., 1.)['low']['reason'] == 'fresh_evidence'


@pytest.mark.parametrize('legacy', [False, True])
@pytest.mark.parametrize('reuse_instance', [False, True])
def test_restart_never_reuses_pre_restart_trial_evidence(legacy, reuse_instance):
  pipe = _evidenced_pipe()
  trial = pipe.recommend(1., 1.)
  state = json.loads(json.dumps(pipe.to_dict()))
  if legacy:
    for pending in state['verify']:
      pending.pop('baseline')
  restored = pipe if reuse_instance else AutoCalPipeline(PLATFORM_GAIN_HIGH)
  restored.from_dict(state)
  assert restored.est.responses.response(0)[0] == 0.
  assert restored.recommend(*trial) == (1., 1.)
  assert restored.verify_result == {0: 'expired', 1: 'expired'}


def test_delay_learning_blocks_even_rollback_writes():
  ctl = AutoCalController(DT)
  params = _MockParams({'FordAngleAutoCal': 1})
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe()
  trial = ctl.pipeline.recommend(1., 1.)
  assert ctl._apply_nudge(trial)
  params.written.clear()
  frame = _frame(10., .004, .004, low=trial[0], high=trial[1])
  for _ in range(100):
    ctl.feed(frame, delay_estimated=False)
  assert not params.written
  assert ctl.pipeline.recovery[0] is not None
  ctl.feed(frame, delay_estimated=True)
  assert params.written['FordLowSpeedFactor_ang'] == 1.
  assert params.written['FordHighSpeedFactor_ang'] == 1.


def test_controller_ages_responses_over_real_inactive_gap():
  ctl = AutoCalController(DT)
  ctl.pipeline = _evidenced_pipe()
  with patch('opendbc.sunnypilot.car.ford.angle_autocal_controller.time.monotonic', side_effect=[10., 611.]):
    ctl.idle()
    ctl.idle()
  assert ctl.pipeline.est.responses.response(0)[0] == 0.


def test_wrong_direction_and_unsteady_response_cannot_verify():
  pipe = _evidenced_pipe()
  pipe._active_half = 0
  trial = pipe.recommend(1., 1.)
  _feed_low(pipe, trial, 15., 1.1, 1.1, kappa=-.004)
  assert pipe.verify[0] is not None  # no left baseline for right turns
  pipe.est.responses.clear()
  baseline_key = int(next(iter(pipe.verify[0]['baseline']['bins'])))
  for ratio in (.5, 1.5):
    pipe.est.responses.add(0, baseline_key, 6., ratio)
  pipe._judge_verifies()
  assert pipe.verify[0] is not None
  assert pipe.ui_state(*trial)['low']['reason'] == 'response_consistency'


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('index', range(5))
def test_nonfinite_sample_cannot_contaminate_window(value, index):
  pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
  args = [10., .002, .002, 1., .05]
  args[index] = value
  assert not pipe.est.add_sample(*args[:4], weight=args[4])
  assert pipe.est.n == 0
  assert pipe.est.responses.response(0)[0] == 0.
