"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Unit tests for LaneCenterTrim (angle-mode lane centering / advanced lane positioning).
#
# Isolated from CarController/LateralAngleExt -- exercises the class directly against a
# minimal fake modelV2-shaped object.

import unittest

from opendbc.sunnypilot.car.ford.lane_center_trim import (
  LaneCenterTrim, _CORRECTION_ROC_PER_TICK, _VEHICLE_HALF_WIDTH_AND_MARGIN_M,
)


class _XY:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Position:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Model:
  def __init__(self, lane_lines, probs, stds, pos_x, pos_y):
    self.laneLines = lane_lines
    self.laneLineProbs = probs
    self.laneLineStds = stds # add stds
    self.position = _Position(pos_x, pos_y)


def _good_model(lane_center_y=0.0, model_y=0.0, width=3.7, probs=(0.9, 0.9, 0.9, 0.9), stds=(0.1, 0.1, 0.1, 0.1)):
  """Confident lane lines (probability + typical width) centered at lane_center_y."""
  xs = list(range(0, 60, 2))
  half = width / 2.0
  lane_lines = [
    _XY(xs, [lane_center_y - half - 3.7] * len(xs)),  # far left (unused)
    _XY(xs, [lane_center_y - half] * len(xs)),         # ego left
    _XY(xs, [lane_center_y + half] * len(xs)),         # ego right
    _XY(xs, [lane_center_y + half + 3.7] * len(xs)),  # far right (unused)
  ]
  pos_x = list(range(0, 60, 2))
  pos_y = [model_y] * len(pos_x)
  return _Model(lane_lines, list(probs), list(stds), pos_x, pos_y)


def _no_lanelines_model(model_y=0.0):
  """No lane lines detected at all (e.g. center-stripe-only road) -- zero probability on both
  ego lines. laneLines arrays still present/well-formed (as modelV2 always publishes them),
  just untrustworthy."""
  return _good_model(model_y=model_y, probs=(0.0, 0.0, 0.0, 0.0))


def _one_sided_model(model_y=0.0):
  """Only the right lane line is confidently detected (e.g. curb on the left, no line there) --
  the real-world case this blend exists for."""
  return _good_model(model_y=model_y, probs=(0.9, 0.05, 0.9, 0.9))


def _single_boundary_model(side, distance, std=0.1, road_edge=False):
  """A reliable boundary on one side, with no opposite lane-center reference."""
  model = _no_lanelines_model()
  xs = model.position.x
  boundary = _XY(xs, [(-1 if side == 0 else 1) * distance] * len(xs))
  if road_edge:
    model.roadEdges = [_XY([], []), _XY([], [])]
    model.roadEdgeStds = [1.0, 1.0]
    model.roadEdges[side] = boundary
    model.roadEdgeStds[side] = std
  else:
    model.laneLines[side + 1] = boundary
    model.laneLineProbs[side + 1] = 0.9
    model.laneLineStds[side + 1] = std
  return model


class TestLaneCenterTrim(unittest.TestCase):
  V_EGO = 15.0  # m/s -- at/above the 9-15 m/s ramp top, full speed authority

  def setUp(self):
    self.trim = LaneCenterTrim()

  def _run(self, model, kappa_cmd=0.0, enabled=True, offset=0.0, gain=1.0, lat_active=True,
            lane_change=False, v_ego=None, iterations=1):
    result = kappa_cmd
    for _ in range(iterations):
      result = self.trim.update(kappa_cmd, model, v_ego if v_ego is not None else self.V_EGO,
                                 enabled, offset, gain, lat_active, lane_change)
    return result

  def test_disabled_no_correction(self):
    model = _good_model()
    self._run(model, enabled=False, offset=1.0, iterations=50)
    self.assertEqual(self.trim.correction, 0.0)

  def test_inactive_no_correction(self):
    model = _good_model()
    self._run(model, lat_active=False, offset=1.0, iterations=50)
    self.assertEqual(self.trim.correction, 0.0)

  def test_lane_change_no_correction(self):
    model = _good_model()
    self._run(model, lane_change=True, offset=1.0, iterations=50)
    self.assertEqual(self.trim.correction, 0.0)

  def test_no_model_no_correction(self):
    self._run(None, offset=1.0, iterations=50)
    self.assertEqual(self.trim.correction, 0.0)

  def test_zero_offset_centered_no_correction(self):
    # Model already tracks lane center exactly; no error to correct.
    model = _good_model(lane_center_y=0.0, model_y=0.0)
    self._run(model, offset=0.0, iterations=50)
    self.assertAlmostEqual(self.trim.correction, 0.0, places=6)

  def test_offset_converges_to_clipped_ceiling(self):
    # A large offset saturates the raw correction against its safety ceiling; full gain and full
    # speed authority means the filtered correction converges to that ceiling.
    model = _good_model()
    self._run(model, offset=5.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.004, places=4)

  def test_gain_scales_correction_linearly(self):
    model = _good_model()
    self._run(model, offset=5.0, gain=0.25, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.001, places=4)

  def test_negative_offset_flips_sign(self):
    model = _good_model()
    self._run(model, offset=-5.0, gain=0.5, iterations=500)
    self.assertAlmostEqual(self.trim.correction, -0.002, places=4)

  def test_speed_ramp_zero_below_9ms(self):
    model = _good_model()
    self._run(model, offset=5.0, gain=1.0, v_ego=5.0, iterations=50)
    self.assertEqual(self.trim.correction, 0.0)

  def test_speed_ramp_partial_between_9_and_15ms(self):
    model = _good_model()
    self._run(model, offset=5.0, gain=0.5, v_ego=12.0, iterations=500)
    # speed_factor at 12 m/s is 0.5 on the [9, 15] -> [0, 1] ramp
    self.assertAlmostEqual(self.trim.correction, 0.001, places=4)

  def test_added_to_kappa_cmd(self):
    model = _good_model()
    result = self._run(model, kappa_cmd=0.01, offset=5.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(result, 0.01 + 0.004, places=4)

  def test_reset_zeroes_filter(self):
    model = _good_model()
    self._run(model, offset=5.0, gain=1.0, iterations=200)
    self.assertNotEqual(self.trim.correction, 0.0)
    self.trim.reset()
    self.assertEqual(self.trim.correction, 0.0)

  def test_disable_mid_stream_resets(self):
    model = _good_model()
    self._run(model, offset=5.0, gain=1.0, iterations=200)
    self.assertNotEqual(self.trim.correction, 0.0)
    self._run(model, enabled=False, offset=5.0, gain=1.0, iterations=1)
    self.assertEqual(self.trim.correction, 0.0)

  # --- Model-position fallback / blend (the point of this class) ---

  def test_no_lanelines_still_applies_offset(self):
    # The scenario this exists for: center-stripe-only road, no lane line on the curbed side.
    # Bad detection must NOT zero out the trim -- the offset bias should still apply, riding on
    # the model's own path, exactly as if lane lines were centered and confident.
    no_lines = _no_lanelines_model()
    self._run(no_lines, offset=5.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.004, places=4)  # same ceiling as good lanelines

  def test_one_sided_laneline_still_applies_offset(self):
    # Only one ego line confidently detected -- min(probL, probR, width_tol) is dragged down by
    # the missing side, same as fully-missing lines.
    one_sided = _one_sided_model()
    self._run(one_sided, offset=5.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.004, places=4)

  def test_no_lanelines_zero_offset_no_correction(self):
    # No lanelines AND no user bias: target collapses to the model's own path, so there's truly
    # nothing to correct (not even a spurious pull toward a phantom lane center).
    no_lines = _no_lanelines_model()
    self._run(no_lines, offset=0.0, gain=1.0, iterations=50)
    self.assertAlmostEqual(self.trim.correction, 0.0, places=6)

  def test_confident_lanelines_pull_toward_laneline_center_even_with_zero_offset(self):
    # Lane center is offset from the model's current path (model drifted left of true center);
    # with full confidence and no user bias, the trim should still pull toward the laneline
    # center -- this is the "full centering" behavior confidence unlocks.
    model = _good_model(lane_center_y=2.0, model_y=0.0)  # model 2m left of true lane center
    self._run(model, offset=0.0, gain=1.0, iterations=500)
    self.assertGreater(self.trim.correction, 0.0)  # pulls right, toward center

  def test_low_confidence_ignores_laneline_center_offset(self):
    # Same lane-position error as above, but lines are unreliable -- with zero user bias the
    # trim must NOT invent a correction from a laneline center it shouldn't trust.
    unreliable = _good_model(lane_center_y=2.0, model_y=0.0, probs=(0.1, 0.1, 0.1, 0.1))
    self._run(unreliable, offset=0.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.0, places=6)

  def test_implausible_width_falls_back_to_model_position(self):
    # A width far outside anything the width-tolerance curve models still degrades confidence
    # smoothly (not a crash / hard reject) and the offset-only fallback still applies.
    weird_width = _good_model(width=15.0)
    self._run(weird_width, offset=5.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.004, places=4)

  def test_no_model_position_hard_resets(self):
    # Unlike laneline-only failures, a missing/invalid model.position leaves no baseline to
    # offset from at all -- this is the one case that still hard-resets to zero.
    broken = _Model(lane_lines=[_XY([], []), _XY([], []), _XY([], []), _XY([], [])],
                     probs=[0.9, 0.9, 0.9, 0.9],stds=[0.1, 0.1, 0.1, 0.1], pos_x=[], pos_y=[])
    self._run(broken, offset=5.0, gain=1.0, iterations=50)
    self.assertEqual(self.trim.correction, 0.0)

  # --- Hardening: guards restored from the reference implementation ---

  def test_narrow_lane_not_centered(self):
    # Two confident stripes 1.5 m apart (double-stripe repaint, gore-point paint) must NOT be
    # treated as a lane to center in: the width-tolerance low edge zeroes the laneline scale,
    # so with no user bias there is no correction at all.
    narrow = _good_model(lane_center_y=1.0, model_y=0.0, width=1.5)
    self._run(narrow, offset=0.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.0, places=6)

  def test_high_std_ignores_laneline_center(self):
    # High positional uncertainty (rain/glare: the model is sure lines exist but not where)
    # must drag centering authority to zero even when probabilities are high.
    blurry = _good_model(lane_center_y=2.0, model_y=0.0, stds=(0.1, 0.6, 0.1, 0.1))
    self._run(blurry, offset=0.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.0, places=6)

  def test_nan_offset_is_inert(self):
    # A corrupt float param must never reach the wire: NaN offset -> no correction, kappa_cmd
    # passes through untouched.
    result = self._run(_good_model(), kappa_cmd=0.01, offset=float("nan"), iterations=10)
    self.assertEqual(self.trim.correction, 0.0)
    self.assertEqual(result, 0.01)

  def test_nan_gain_is_inert(self):
    result = self._run(_good_model(), kappa_cmd=0.01, offset=0.3, gain=float("nan"), iterations=10)
    self.assertEqual(self.trim.correction, 0.0)
    self.assertEqual(result, 0.01)

  def test_nonfinite_speed_is_inert(self):
    for speed in (float("nan"), float("inf")):
      with self.subTest(speed=speed):
        result = self._run(_good_model(), kappa_cmd=0.01, offset=0.3, v_ego=speed)
        self.assertEqual(self.trim.correction, 0.0)
        self.assertEqual(result, 0.01)

  # A credible single boundary must constrain bias even when lane-center confidence is zero.

  def test_close_single_boundary_blocks_only_trim_toward_it(self):
    for side in (0, 1):
      direction = -1 if side == 0 else 1
      model = _single_boundary_model(side, 0.8)
      with self.subTest(side=side):
        self.trim.reset()
        self._run(model, offset=direction * 0.3, iterations=500)
        self.assertEqual(self.trim.correction, 0.0)
        # A path already outside the envelope does not trigger invented recovery steering.
        result = self._run(model, kappa_cmd=0.01, offset=0.0)
        self.assertEqual(result, 0.01)
        self._run(model, offset=-direction * 0.3, iterations=500)
        self.assertLess(self.trim.correction * direction, 0.0)

  def test_single_boundary_caps_bias_to_available_clearance(self):
    for side in (0, 1):
      direction = -1 if side == 0 else 1
      with self.subTest(side=side):
        self.trim.reset()
        model = _single_boundary_model(side, 1.3)
        self._run(model, offset=direction * 5.0, iterations=500)
        expected = direction * 2.0 * (1.3 - _VEHICLE_HALF_WIDTH_AND_MARGIN_M - 0.1) / self.V_EGO ** 2
        self.assertAlmostEqual(self.trim.correction, expected, places=9)

  def test_higher_boundary_uncertainty_leaves_more_margin(self):
    corrections = []
    for std in (0.0, 0.2):
      self.trim.reset()
      self._run(_single_boundary_model(1, 1.4, std=std), offset=5.0, iterations=500)
      corrections.append(self.trim.correction)
    self.assertGreater(corrections[0], corrections[1])
    self.assertGreater(corrections[1], 0.0)

  def test_credible_road_edge_works_without_lane_lines(self):
    for side in (0, 1):
      with self.subTest(side=side):
        self.trim.reset()
        model = _single_boundary_model(side, 0.8, road_edge=True)
        self._run(model, offset=-0.3 if side == 0 else 0.3, iterations=500)
        self.assertEqual(self.trim.correction, 0.0)

  def test_boundary_uncertainty_and_probability_must_be_credible(self):
    for road_edge in (False, True):
      for std in (0.6, float("nan"), -0.1):
        with self.subTest(road_edge=road_edge, std=std):
          self.trim.reset()
          model = _single_boundary_model(1, 0.8, std=std, road_edge=road_edge)
          self._run(model, offset=0.3, iterations=500)
          self.assertGreater(self.trim.correction, 0.0)
    for prob in (0.2, float("nan"), float("inf"), 1.1):
      with self.subTest(prob=prob):
        self.trim.reset()
        model = _single_boundary_model(1, 0.8)
        model.laneLineProbs[2] = prob
        self._run(model, offset=0.3, iterations=500)
        self.assertGreater(self.trim.correction, 0.0)

  def test_invalid_one_side_does_not_hide_valid_opposite_boundary(self):
    model = _single_boundary_model(1, 0.8)
    model.laneLineStds[1] = float("nan")
    model.laneLineProbs[1] = float("nan")
    self._run(model, offset=0.3, iterations=500)
    self.assertEqual(self.trim.correction, 0.0)

  def test_boundary_is_not_extrapolated_outside_its_coverage(self):
    model = _single_boundary_model(1, 0.8, road_edge=True)
    model.roadEdges[1] = _XY([40.0, 50.0], [0.8, 0.8])
    self._run(model, offset=0.3, iterations=500)
    self.assertGreater(self.trim.correction, 0.0)

  def test_malformed_boundaries_do_not_poison_trim(self):
    for xs, ys in (([0, 5, 4], [0.8] * 3), ([0, 5], [0.8]),
                   ([0, 5], [float("nan"), 0.8]), ([], [])):
      with self.subTest(xs=xs, ys=ys):
        self.trim.reset()
        model = _single_boundary_model(1, 0.8, road_edge=True)
        model.roadEdges[1] = _XY(xs, ys)
        self._run(model, offset=0.3, iterations=500)
        self.assertGreater(self.trim.correction, 0.0)

  def test_curve_near_boundary_checked_before_clear_lookahead(self):
    for side in (0, 1):
      direction = -1 if side == 0 else 1
      with self.subTest(side=side):
        self.trim.reset()
        model = _single_boundary_model(side, 3.0, road_edge=True)
        xs = model.position.x
        model.position.y = [direction * 0.002 * x ** 2 for x in xs]
        # A tight section near the car, followed by ample clearance at lookahead.
        model.roadEdges[side].y = [y + direction * (0.8 if x < 8 else 3.0)
                                    for x, y in zip(xs, model.position.y)]
        self._run(model, offset=direction * 0.3, iterations=500)
        self.assertEqual(self.trim.correction, 0.0)

  def test_curved_path_uses_normal_clearance(self):
    model = _single_boundary_model(1, 1.3)
    model.position.y = [0.7 * x for x in model.position.x]
    model.laneLines[2].y = [y + 1.3 for y in model.position.y]
    self._run(model, offset=0.3, iterations=500)
    # 1.3 m in y is less than the 1.18 m envelope measured normal to this sloping path.
    self.assertEqual(self.trim.correction, 0.0)

  def test_conflicting_narrow_boundaries_do_not_create_steering(self):
    for offset in (-0.3, 0.0, 0.3):
      with self.subTest(offset=offset):
        self.trim.reset()
        self._run(_good_model(width=1.5), offset=offset, iterations=500)
        self.assertEqual(self.trim.correction, 0.0)

  def test_boundary_appearance_and_dropout_preserve_slew_limit(self):
    self._run(_no_lanelines_model(), offset=0.3, iterations=500)
    close = _single_boundary_model(1, 0.8)
    before = self.trim.correction
    self._run(close, offset=0.3)
    self.assertLess(self.trim.correction, before)
    self.assertLessEqual(abs(self.trim.correction - before), _CORRECTION_ROC_PER_TICK + 1e-9)
    self._run(close, offset=0.3, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.0, places=9)
    before = self.trim.correction
    self._run(_no_lanelines_model(), offset=0.3)
    self.assertGreater(self.trim.correction, before)
    self.assertLessEqual(abs(self.trim.correction - before), _CORRECTION_ROC_PER_TICK + 1e-9)

  def test_boundary_does_not_interfere_during_lane_change(self):
    model = _single_boundary_model(1, 0.8)
    result = self._run(model, kappa_cmd=0.01, offset=-0.3, lane_change=True, iterations=50)
    self.assertEqual(result, 0.01)
    self.assertEqual(self.trim.correction, 0.0)

  # --- Rate-of-change limit: smooths confidence transitions (merge lanes, lines dropping out) ---

  def test_correction_rate_limited_on_confidence_jump(self):
    # Converge to a strong positive correction under confident, off-center lane lines.
    centered_confident = _good_model(lane_center_y=2.0, model_y=0.0)
    self._run(centered_confident, offset=0.0, gain=1.0, iterations=500)
    before = self.trim.correction
    self.assertAlmostEqual(before, 0.004, places=3)

    # Lane lines vanish entirely in the very next frame (e.g. crossing into a too-wide merge
    # lane) while a negative user offset now drives the fallback target the opposite way -- a
    # large, instantaneous target swing (+0.004 -> -0.004 worth of target).
    no_lines = _no_lanelines_model(model_y=0.0)
    after = self.trim.update(0.0, no_lines, self.V_EGO, True, -5.0, 1.0, True, False)

    # The correction must not have snapped toward the new target -- bounded to roughly one
    # tick's worth of rate-of-change, regardless of how far the target actually moved.
    self.assertLessEqual(abs(after - before), _CORRECTION_ROC_PER_TICK + 1e-9)

  def test_correction_eventually_converges_despite_rate_limit(self):
    # The rate limit paces the transition but must not prevent it from completing.
    centered_confident = _good_model(lane_center_y=2.0, model_y=0.0)
    self._run(centered_confident, offset=0.0, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, 0.004, places=3)

    no_lines = _no_lanelines_model(model_y=0.0)
    self._run(no_lines, offset=-5.0, gain=1.0, iterations=1000)
    self.assertAlmostEqual(self.trim.correction, -0.004, places=3)

  def test_fallback_bias_is_position_independent_by_design(self):
    # Pins the fallback semantics deliberately: with no lanelines, error == offset regardless of
    # where the model path is (target moves with the baseline), i.e. the bias is a constant push,
    # not a position controller. If this ever changes, it should change on purpose.
    a = _no_lanelines_model(model_y=0.0)
    b = _no_lanelines_model(model_y=-3.0)
    self._run(a, offset=0.3, gain=1.0, iterations=500)
    correction_a = self.trim.correction
    self.trim.reset()
    self._run(b, offset=0.3, gain=1.0, iterations=500)
    self.assertAlmostEqual(self.trim.correction, correction_a, places=9)


if __name__ == "__main__":
  unittest.main()
