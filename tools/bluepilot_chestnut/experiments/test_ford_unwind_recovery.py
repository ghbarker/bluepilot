"""Properties and unresolved acceptance conditions of offline candidate two."""
import math

import pytest

from tools.bluepilot_chestnut.experiments.ford_unwind_recovery import PlannerUnwindRecovery


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('blend', [0., .5, 1.])
def test_normal_entry_and_constant_curve_unchanged(direction, blend):
  taper = PlannerUnwindRecovery()
  for desired in [.0001, .0002, .001, .003, .004] + [.004] * 20:
    desired *= direction
    nominal = desired + direction * blend * .002
    assert taper.update(desired, nominal) == pytest.approx(nominal)


@pytest.mark.parametrize('direction', [-1, 1])
def test_constant_retaining_contribution_reaches_zero_continuously(direction):
  taper = PlannerUnwindRecovery()
  previous = None
  for i in range(101):
    desired = direction * .004 * (1. - i / 100)
    result = taper.update(desired, desired + direction * .002)
    if previous is not None:
      assert abs(result - previous) <= .000061
    previous = result
  assert result == 0.


@pytest.mark.parametrize('direction', [-1, 1])
def test_tiny_dips_do_not_hard_cap_growing_prediction(direction):
  taper = PlannerUnwindRecovery()
  for desired, contribution in [(.004, .0001), (.003999, .0002), (.003998, .0004)] + [(.003998, .003)] * 200:
    result = taper.update(direction * desired, direction * (desired + contribution))
    assert abs(result - direction * (desired + contribution)) <= contribution * .0005 + 1e-12


@pytest.mark.parametrize('direction', [-1, 1])
def test_assisting_prediction_is_unchanged(direction):
  taper = PlannerUnwindRecovery()
  for desired in [.004, .003, .002, .001]:
    nominal = direction * (desired - .0005)
    assert taper.update(direction * desired, nominal) == pytest.approx(nominal)


@pytest.mark.parametrize('direction', [-1, 1])
def test_shallower_new_entry_recovers_without_reaching_old_peak(direction):
  taper = PlannerUnwindRecovery()
  for desired in [.006, .004, .002, .001, .0015, .002, .0025, .003]:
    result = taper.update(direction * desired, direction * (desired + .001))
  assert result == pytest.approx(direction * .004)
  assert taper.budget == 0.


@pytest.mark.parametrize('direction', [-1, 1])
def test_crossed_curve_unwind_uses_new_direction(direction):
  taper = PlannerUnwindRecovery()
  taper.update(direction * .003, direction * .004)
  for desired, predicted in [(.002, .003), (.001, .002), (0., .001), (-.001, 0.), (-.002, -.001), (-.003, -.001)]:
    taper.update(direction * desired, direction * .5 * (desired + predicted))
  desired, predicted = -.0025, -.001
  nominal = direction * .5 * (desired + predicted)
  assert taper.update(direction * desired, nominal) == pytest.approx(nominal)
  assert taper.direction == -direction


@pytest.mark.parametrize('direction', [-1, 1])
def test_recovery_uses_net_progress_not_accumulated_positive_steps(direction):
  taper = PlannerUnwindRecovery()
  taper.update(direction * .004, direction * .006)
  at_trough = taper.update(direction * .002, direction * .004)
  samples = []
  for _ in range(100):
    samples.append(taper.update(direction * .0021, direction * .0041))
    assert taper.update(direction * .002, direction * .004) == pytest.approx(at_trough)
  assert max(samples) == min(samples)
  assert taper.budget == pytest.approx(.001)


@pytest.mark.parametrize('direction', [-1, 1])
def test_recovery_interruption_carries_budget_when_raw_prediction_changes(direction):
  taper = PlannerUnwindRecovery()
  taper.update(direction * .004, direction * .006)
  taper.update(direction * .002, direction * .004)
  taper.update(direction * .0021, direction * .0061)
  # Return to the same trough with a larger optional contribution. The existing
  # budget remains .001; recomputing a 50% taper would jump it to .002.
  at_trough = taper.update(direction * .002, direction * .006)
  assert taper.budget == pytest.approx(.001)
  epsilon = 1e-8
  below_trough = taper.update(direction * (.002 - epsilon), direction * (.006 - epsilon))
  assert taper.budget == pytest.approx(.001 + .003 * epsilon / .002)
  assert abs(below_trough - at_trough) < 3 * epsilon


@pytest.mark.parametrize('direction', [-1, 1])
def test_temporary_assisting_prediction_does_not_clear_recovery_budget(direction):
  taper = PlannerUnwindRecovery()
  taper.update(direction * .004, direction * .006)
  taper.update(direction * .002, direction * .004)
  taper.update(direction * .0021, direction * .0041)
  budget = taper.budget
  for contribution in [0., -.001, 0., .002]:
    nominal = direction * (.0021 + contribution)
    result = taper.update(direction * .0021, nominal)
    assert taper.budget == pytest.approx(budget)
    if contribution <= 0.:
      assert result == pytest.approx(nominal)


@pytest.mark.parametrize('direction', [-1, 1])
def test_zero_crossing_oscillations_do_not_earn_cumulative_release(direction):
  taper = PlannerUnwindRecovery()
  taper.update(direction * .003, direction * .005)
  taper.update(0., direction * .002)
  for _ in range(100):
    for desired in [-.0001, 0., .0001, 0.]:
      taper.update(direction * desired, direction * (desired + .002))
      assert taper.budget == pytest.approx(.002 - abs(desired))
  assert taper.direction == direction


@pytest.mark.parametrize('direction', [-1, 1])
def test_output_neutral_budget_exhaustion(direction):
  taper = PlannerUnwindRecovery()
  taper.update(direction * .004, direction * .006)
  taper.update(direction * .002, direction * .004)
  previous = None
  for i in range(101):
    desired = .002 + i * .00001
    result = taper.update(direction * desired, direction * (desired + .002))
    if previous is not None:
      assert abs(result - previous) <= .000020001
    previous = result
  assert result == pytest.approx(direction * .005)
  assert taper.budget == 0.
  assert taper.scale == 1.


@pytest.mark.parametrize('direction', [-1, 1])
def test_correction_always_remains_between_planner_and_nominal(direction):
  taper = PlannerUnwindRecovery()
  for index in range(500):
    desired = direction * .004 * math.sin(index / 23.)
    nominal = desired + .003 * math.sin(index / 7.)
    result = taper.update(desired, nominal)
    assert min(desired, nominal) - 1e-12 <= result <= max(desired, nominal) + 1e-12
    assert 0. <= taper.scale <= 1.
    assert math.isfinite(result)


@pytest.mark.parametrize('invalid', [math.inf, -math.inf, math.nan])
@pytest.mark.parametrize('field', ['desired', 'nominal'])
def test_invalid_input_clears_all_state(invalid, field):
  taper = PlannerUnwindRecovery()
  taper.update(.004, .006)
  taper.update(.002, .004)
  with pytest.raises(ValueError):
    taper.update(**{'desired': .001, 'nominal': .004, field: invalid})
  assert taper == PlannerUnwindRecovery()


@pytest.mark.parametrize('invalid', [0., -1., math.inf, math.nan])
def test_restore_assumption_must_be_positive_and_finite(invalid):
  with pytest.raises(ValueError):
    PlannerUnwindRecovery(restore_gain=invalid)


def test_reset_preserves_only_explicit_restore_assumption():
  taper = PlannerUnwindRecovery(restore_gain=.75)
  taper.update(.004, .006)
  taper.update(.002, .004)
  taper.reset()
  assert taper == PlannerUnwindRecovery(restore_gain=.75)


@pytest.mark.xfail(strict=True, reason='BLOCKS RELEASE: growing raw prediction can overpower proportional withdrawal')
def test_acceptance_raw_growth_cannot_deepen_request_after_planner_reductions():
  taper = PlannerUnwindRecovery()
  taper.update(.004, .005)
  first = taper.update(.0035, .0045)
  second = taper.update(.003, .007)
  assert second <= first


@pytest.mark.xfail(strict=True, reason='BLOCKS RELEASE: restored growing raw prediction can oppose the new planner direction')
def test_acceptance_crossing_restoration_does_not_reintroduce_old_turn():
  taper = PlannerUnwindRecovery()
  taper.update(.003, .004)
  taper.update(0., .002)
  result = taper.update(-.001, .003)
  assert result <= 0.
