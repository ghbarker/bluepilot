"""Fault-injection tests for gain writes and the saved recovery record."""
import json

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.tests.test_angle_autocal import DT, PLATFORM_GAIN_HIGH, _MockParams, _evidenced_pipe, _frame

FACTOR_KEYS = ('FordLowSpeedFactor_ang', 'FordHighSpeedFactor_ang')
STATE = 'FordAngleAutoCalState'


class FaultParams(_MockParams):
  def __init__(self, fault_key=None, mode='raise'):
    super().__init__({'FordAngleAutoCal': True, STATE: '', **dict.fromkeys(FACTOR_KEYS, 1.)})
    self.fault_key = fault_key
    self.mode = mode
    self.calls = []
    self.disk = dict(self.values)

  def put(self, key, value, block=False):
    self.calls.append((key, block))
    if key == self.fault_key:
      if self.mode == 'raise':
        raise OSError('injected write failure')
      if self.mode == 'drop':
        return  # native Params can return a failure without a Python exception
    super().put(key, value, block)
    if block:
      self.disk[key] = value
    if key == self.fault_key and self.mode == 'after_write':
      raise OSError('injected failure after rename')
    if key == self.fault_key and self.mode == 'power_loss':
      raise SystemExit('simulated process death after factor persisted')


def armed(params):
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe()
  return ctl


@pytest.mark.parametrize('mode', ['raise', 'drop'])
def test_cannot_change_factors_without_saved_recovery_record(mode):
  params = FaultParams(STATE, mode)
  ctl = armed(params)
  trial = ctl.pipeline.recommend(1., 1.)
  assert not ctl._apply_nudge(trial)
  assert all(params.values[k] == 1. for k in FACTOR_KEYS)
  assert not any(k in FACTOR_KEYS for k, _ in params.calls)


@pytest.mark.parametrize('mode', ['raise', 'drop', 'after_write'])
def test_partial_pair_write_cannot_be_mistaken_for_manual_edit(mode):
  params = FaultParams(FACTOR_KEYS[1], mode)
  ctl = armed(params)
  trial = ctl.pipeline.recommend(1., 1.)
  assert not ctl._apply_nudge(trial)
  actual = tuple(params.values[k] for k in FACTOR_KEYS)
  assert actual[0] == trial[0]
  ctl.poll_params(params, *actual, PLATFORM_GAIN_HIGH)
  assert any(ctl.pipeline.verify.values()) or any(ctl.pipeline.recovery.values())
  assert ctl._last_written == actual


@pytest.mark.parametrize('half', [0, 1])
def test_power_loss_after_factor_write_preserves_bounded_rollback(half):
  params = FaultParams(FACTOR_KEYS[half], 'power_loss')
  ctl = armed(params)
  ctl.pipeline._active_half = half
  trial = ctl.pipeline.recommend(1., 1.)
  with pytest.raises(SystemExit):
    ctl._apply_nudge(trial)
  rebooted = _MockParams(dict(params.disk))
  actual = tuple(rebooted.values[k] for k in FACTOR_KEYS)
  assert actual[half] == trial[half]
  restored = AutoCalController(DT)
  restored.poll_params(rebooted, *actual, PLATFORM_GAIN_HIGH)
  assert restored.pipeline.recovery[half] is not None
  assert restored.pipeline.recommend(*actual) == (1., 1.)
  restored.feed(_frame(10., .004, .004, low=actual[0], high=actual[1]), delay_estimated=False)
  assert all(rebooted.values[k] == actual[h] for h, k in enumerate(FACTOR_KEYS))


def test_state_checkpoints_cannot_queue_old_state_over_a_new_trial():
  params = FaultParams()
  ctl = armed(params)
  ctl._save('collecting', (1., 1.))
  trial = ctl.pipeline.recommend(1., 1.)
  assert ctl._apply_nudge(trial)
  assert all(block for key, block in params.calls if key == STATE)
  assert json.loads(params.disk[STATE])['pipe']['verify'][0]


def test_manual_factor_change_between_frame_and_write_is_preserved():
  params = FaultParams()
  ctl = armed(params)
  trial = ctl.pipeline.recommend(1., 1.)
  params.values[FACTOR_KEYS[1]] = 1.12
  assert not ctl._apply_nudge(trial)
  assert tuple(params.values[k] for k in FACTOR_KEYS) == (1., 1.12)
  assert not any(ctl.pipeline.verify.values())


def test_manual_edit_after_saved_trial_does_not_leave_restart_stuck_in_recovery():
  params = FaultParams()
  ctl = armed(params)
  trial = ctl.pipeline.recommend(1., 1.)
  assert ctl._apply_nudge(trial)
  params.values[FACTOR_KEYS[0]] = 1.12  # driver changes the saved factor before restart
  restarted = AutoCalController(DT)
  restarted.poll_params(params, 1.12, trial[1], PLATFORM_GAIN_HIGH)
  assert not any(restarted.pipeline.verify.values())
  assert not any(restarted.pipeline.recovery.values())
  assert params.values[FACTOR_KEYS[0]] == 1.12


@pytest.mark.parametrize('trigger', [STATE, FACTOR_KEYS[0]])
def test_manual_edit_during_checkpoint_or_previous_write_is_preserved(trigger):
  class EditDuringWrite(FaultParams):
    def put(self, key, value, block=False):
      super().put(key, value, block)
      if key == trigger:
        self.values[FACTOR_KEYS[1]] = 1.12
  params = EditDuringWrite()
  ctl = armed(params)
  trial = ctl.pipeline.recommend(1., 1.)
  assert not ctl._apply_nudge(trial)
  assert params.values[FACTOR_KEYS[1]] == 1.12
  assert not any(k == FACTOR_KEYS[1] for k, _ in params.calls)
  assert not any(ctl.pipeline.verify.values())


def test_disable_during_checkpoint_cannot_be_followed_by_factor_write():
  class DisableDuringSave(FaultParams):
    def put(self, key, value, block=False):
      super().put(key, value, block)
      if key == STATE and value:
        self.values['FordAngleAutoCal'] = False
  params = DisableDuringSave()
  ctl = armed(params)
  ctl.feed(_frame(10., .004, .004 / 1.1), delay_estimated=True)
  assert not ctl.enabled and ctl.pipeline is None
  assert all(params.values[k] == 1. for k in FACTOR_KEYS)
  assert params.values[STATE] == ''
