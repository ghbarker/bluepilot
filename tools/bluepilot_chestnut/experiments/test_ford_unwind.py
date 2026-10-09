"""Offline candidate properties and explicit, blocking acceptance failures."""
import math

import pytest

from tools.bluepilot_chestnut.experiments.ford_unwind import PlannerUnwindTaper


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('blend', [0., .5, 1.])
def test_entry_and_constant_curve_unchanged(direction, blend):
  taper = PlannerUnwindTaper()
  for desired in [.0001, .0002, .001, .003, .004] + [.004] * 20:
    desired *= direction
    nominal = desired + direction * blend * .002
    assert taper.update(desired, nominal) == pytest.approx(nominal)


@pytest.mark.parametrize('direction', [-1, 1])
def test_constant_retaining_prediction_fades_continuously_to_zero(direction):
  taper = PlannerUnwindTaper()
  previous = None
  for i in range(101):
    desired = direction * .004 * (1 - i / 100)
    result = taper.update(desired, desired + direction * .002)
    assert desired * direction <= result * direction <= (desired + direction * .002) * direction + 1e-12
    if previous is not None:
      assert abs(result - previous) <= .000061
    previous = result
  assert result == 0.


@pytest.mark.parametrize('direction', [-1, 1])
def test_prediction_helping_existing_turn_unwind_is_unchanged(direction):
  taper = PlannerUnwindTaper()
  for desired in [.004, .003, .002, .001]:
    nominal = direction * (desired - .0005)
    assert taper.update(direction * desired, nominal) == pytest.approx(nominal)


@pytest.mark.parametrize('direction', [-1, 1])
def test_tiny_dips_and_growing_prediction_have_proportionately_tiny_effect(direction):
  taper = PlannerUnwindTaper()
  for desired, contribution in [(.004, .0001), (.003999, .0002), (.003998, .0004)] + [(.003998, .003)] * 200:
    result = taper.update(direction * desired, direction * (desired + contribution))
    assert abs(result - direction * (desired + contribution)) <= contribution * .0005 + 1e-12


@pytest.mark.parametrize('direction', [-1, 1])
def test_repeated_rebounds_do_not_ratchet_scale(direction):
  taper = PlannerUnwindTaper()
  taper.update(direction * .004, direction * .006)
  outputs = []
  for _ in range(100):
    outputs.append(taper.update(direction * .003, direction * .005))
    taper.update(direction * .0035, direction * .0055)
  assert max(outputs) == min(outputs)
  assert taper.update(direction * .004, direction * .006) == pytest.approx(direction * .006)


@pytest.mark.parametrize('invalid', [math.inf, -math.inf, math.nan])
@pytest.mark.parametrize('field', ['desired', 'nominal'])
def test_invalid_input_clears_state(invalid, field):
  taper = PlannerUnwindTaper()
  taper.update(.004, .006)
  args = {'desired': .003, 'nominal': .005, field: invalid}
  with pytest.raises(ValueError):
    taper.update(**args)
  assert taper.peak == 0.
  assert taper.scale == 1.


def test_reset_discards_previous_episode():
  taper = PlannerUnwindTaper()
  taper.update(.004, .006)
  taper.update(.001, .003)
  taper.reset()
  assert taper.update(.001, .003) == pytest.approx(.003)


@pytest.mark.xfail(strict=True, reason='BLOCKS RELEASE: rising raw contribution can overpower the proportional taper')
def test_acceptance_raw_growth_can_overpower_a_planner_reduction():
  taper = PlannerUnwindTaper()
  first = taper.update(.004, .005)
  second = taper.update(.003, .007)
  assert second <= first


@pytest.mark.xfail(strict=True, reason='BLOCKS RELEASE: old turn direction can suppress assistance on the next curve')
def test_acceptance_new_curve_unwind_does_not_keep_old_direction():
  taper = PlannerUnwindTaper()
  taper.update(.003, .004)
  for desired, predicted in [(.002, .003), (.001, .002), (0., .001), (-.001, 0.), (-.002, -.001), (-.003, -.001)]:
    taper.update(desired, .5 * (desired + predicted))
  desired, predicted = -.0025, -.001
  nominal = .5 * (desired + predicted)
  assert taper.update(desired, nominal) == pytest.approx(nominal)


@pytest.mark.xfail(strict=True, reason='BLOCKS RELEASE: a shallower subsequent same-direction curve inherits the old peak')
def test_acceptance_new_shallower_entry_regains_full_prediction():
  taper = PlannerUnwindTaper()
  for desired in [.006, .004, .002, .001, .0015, .002, .0025, .003]:
    result = taper.update(desired, desired + .001)
  assert result == pytest.approx(.004)
