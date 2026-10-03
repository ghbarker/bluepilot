"""
BluePilot: angle-mode "advanced lane positioning" -- a curvature-domain trim toward true
lane-line center (plus an optional user left/right bias), for use with path_angle-primary
control (``lateral_angle_ext.py``).

Under curvature-primary control, lane positioning rode on a second DBC signal (path_angle as
a heading trim, c3) layered on top of curvature (c1, the primary). In angle mode, path_angle
IS the primary signal -- there is no separate trim slot on the wire. An earlier attempt ported
the old PID controller as an additive delta on the *final* path_angle value and it never tracked
lane center correctly: path_angle there is a derived quantity (``kappa_cmd * v_ego *
curvature_factor``), so an additive trim in that domain has the wrong (inverted) speed-dependence
for a lane-centering nudge, and it bypasses every limiter angle mode already applies to
``kappa_cmd`` (deviation clip, PSCM-saturation clamp, DBC clip, soft ROC).

This class instead computes a small correction in the **curvature domain**, meant to be added to
``kappa_cmd`` before any of those limiters run -- so the correction automatically inherits all of
them, with no new safety-relevant code path. This mirrors where a GM-focused fork (StarPilot,
u/jc01rho -- ``selfdrive/controls/lib/drive_helpers.py::LaneCenteringController`` on
``feat/lane-centering-camera-offset``) places their own lane-centering trim: on the universal
``desired_curvature`` signal, upstream of any car-specific actuator conversion. The geometry here
(``raw_correction = 2*error / lookahead**2``) is the same ``y ~= 1/2 * kappa * x**2`` relation
BluePilot already uses for PSCM d_ref / path_angle elsewhere in this package, solved for the
curvature that would eliminate a lateral position error at the lookahead distance.

**Model-position fallback (blend, not hard gate)**: mirrors ``lateral_curv_ext.py``'s
``path_offset`` blend of ``path_offset_position`` (the model's own predicted path -- a "no
correction" baseline) vs. ``path_offset_lanelines`` (the geometric lane center), weighted by a
laneline confidence score. The *target* we correct toward is that same blend here, so:

- Good lane lines (confidence -> 1): target -> lane-line center + offset (full centering + bias).
- No/poor lane lines (confidence -> 0): target -> the model's own current path + offset, subject
  to the independent boundary checks below. A missing opposite lane line must not discard a
  credible near-side line or road edge and allow the user's bias to push toward it.

**Boundary-limited trim**: each credible lane line or road edge independently limits the added
correction, using model-path clearance over the near/lookahead interval. The clearance envelope
includes nominal vehicle half-width, a margin and positional uncertainty. This only attenuates
the optional trim; it neither overrides the planner nor creates an evasive steering command when
the model path is already outside the envelope. Existing trim unwinds at the existing slew limit.
It is not collision avoidance or a guarantee of physical clearance: the model, vehicle response
and downstream steering limits still determine the actual trajectory.

Confidence uses the exact formula and breakpoints ``lateral_curv_ext.py`` already uses for
``laneline_confidence`` / ``min_laneline_confidence_bp`` (smooth width-tolerance + laneline probs,
not a hard cutoff) -- reusing an already-tuned, production formula instead of inventing a second one.
"""
import numpy as np
from numpy import interp

# Laneline confidence blend -- ported verbatim from lateral_curv_ext.py's path_offset blend
# (laneline_width_tolerance / min_laneline_confidence_bp) so both control schemes agree on what
# "good lane lines" means.
_WIDTH_TOLERANCE_BP = (2.4, 2.8, 3.75, 4.25)  # m - the floor is added from StarPilot
_WIDTH_TOLERANCE_V = (0.0, 0.81, 0.81, 0.59) # Adds StarPilot's low edge
_STD_TOLERANCE_BP = (0.3, 0.5) #0.3 is from StarPilot to define bad references
_STD_TOLERANCE_V = (0.81, 0.0) #0.81 is a known good value and allows it to fade to zero as std increases
_CONFIDENCE_BP = (0.6, 0.8)
_CONFIDENCE_V = (0.0, 1.0)

# Speed ramp: reuses curvature-mode's already-tuned centering-authority envelope
# (LC_PID_speed_bp/v in lateral_curv_ext.py) instead of inventing a new one.
_SPEED_RAMP_BP = (0.0, 9.0, 15.0)  # m/s
_SPEED_RAMP_V = (0.0, 0.0, 1.0)

# Lookahead distance for the position-error sample, clipped to a sane range (m).
_LOOKAHEAD_MIN_M = 8.0
_LOOKAHEAD_MAX_M = 35.0

# Hard safety ceiling on the raw correction magnitude (1/m) -- fixed, not a "feel" knob, same
# treatment as _PSCM_SAT_UNWIND_RATE / _soft_roc in lateral_angle_ext.py.
_MAX_RAW_CORRECTION = 0.004

# First-order smoothing time constant (s) -- avoids abrupt jumps in the trim.
_SMOOTH_TAU_S = 0.4

# Rate-of-change limit on the applied correction (1/m per 20 Hz tick), independent of the
# smoothing filter above. The filter alone still has its *fastest* slew immediately after a
# target jump -- its per-tick step is proportional to how far the target moved, so a large,
# sudden confidence swing (e.g. crossing into/out of a too-wide merge lane, or lane lines
# dropping out entirely while the model's own path and the laneline center disagree) can still
# produce a big single-tick correction change even though it's technically "smoothed". This caps
# that step explicitly, the same way path_angle's own soft ROC (lateral_angle_ext.py) and
# curvature mode's LC_path_angle_ROC (lateral_curv_ext.py) rate-limit their outputs. At this
# rate, crossing the full -_MAX_RAW_CORRECTION..+_MAX_RAW_CORRECTION span takes ~2.7s; a more
# typical confidence-transition jump (a fraction of that) resolves proportionally faster.
_CORRECTION_ROC_PER_TICK = 0.00015

# Nominal half-width plus margin, matching the 1.08 m envelope already used by LDW and RELC.
# This is a conservative trim constraint, not a measured per-vehicle collision footprint.
_VEHICLE_HALF_WIDTH_AND_MARGIN_M = 1.08
_BOUNDARY_MIN_PROB = 0.8
_BOUNDARY_MAX_STD_M = 0.3
_BOUNDARY_NEAR_M = 2.0
_BOUNDARY_SAMPLES = 6


class LaneCenterTrim:
  def __init__(self):
    self._correction = 0.0

  def reset(self) -> None:
    self._correction = 0.0

  def update(self, kappa_cmd: float, model, v_ego: float, enabled: bool, offset: float,
             gain: float, lat_active: bool, lane_change: bool) -> float:
    """Returns ``kappa_cmd``, nudged toward (lane-blend target + ``offset``) when active.

    ``offset`` (m): positive shifts the target right, negative left (same sign convention as
    curvature mode's ``custom_path_offset_curv``). Applied even without a lane-center estimate,
    but constrained by any independently credible boundary -- see module docstring.
    ``gain`` (0.0-1.0): user-tunable authority -- how much of the (already magnitude-clipped)
    raw correction is actually applied. 0 disables the trim's effect without disabling detection.

    The applied correction is both exponentially smoothed (_SMOOTH_TAU_S) and rate-limited
    (_CORRECTION_ROC_PER_TICK) -- see the constants above -- so a lane-line-confidence
    transition (e.g. a merge lane too wide to be trusted, then narrowing back into range) eases
    into its new target instead of snapping.
    """
    if not enabled or not lat_active or lane_change or model is None:
      self.reset()
      return kappa_cmd

    if not (np.isfinite(v_ego) and np.isfinite(offset) and np.isfinite(gain)):
      self.reset()
      return kappa_cmd

    speed_factor = float(interp(v_ego, _SPEED_RAMP_BP, _SPEED_RAMP_V))
    if speed_factor <= 0.0:
      self.reset()
      return kappa_cmd

    valid, raw = self._raw_correction(model, v_ego, offset)
    if not valid:
      # Only true failure case now: model.position itself is unusable, so there's no baseline
      # to offset from at all (see _raw_correction). Lane-line-only failures fall back to the
      # model-position blend instead of landing here.
      self.reset()
      return kappa_cmd

    target = float(np.clip(raw, -_MAX_RAW_CORRECTION, _MAX_RAW_CORRECTION)) * float(np.clip(gain, 0.0, 1.0)) * speed_factor
    alpha = 1.0 - np.exp(-0.05 / _SMOOTH_TAU_S)  # BluePilot lateral tick is 20 Hz (dt=0.05s)
    filtered = float(alpha * target + (1.0 - alpha) * self._correction)
    # Rate-of-change limit -- see _CORRECTION_ROC_PER_TICK. Bounds the correction's own per-tick
    # delta on top of the exponential filter above, so a lane-line-confidence transition can't
    # snap the trim toward its new target in one or two frames.
    self._correction = float(np.clip(filtered, self._correction - _CORRECTION_ROC_PER_TICK,
                                      self._correction + _CORRECTION_ROC_PER_TICK))
    return kappa_cmd + self._correction

  @property
  def correction(self) -> float:
    """Telemetry: last applied correction (1/m)."""
    return self._correction

  def _raw_correction(self, model, v_ego: float, offset: float) -> tuple[bool, float]:
    try:
      pos_x = np.asarray(model.position.x, dtype=float)
      pos_y = np.asarray(model.position.y, dtype=float)
      if pos_x.size < 2 or pos_x.size != pos_y.size:
        return False, 0.0
      if not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all() and np.all(np.diff(pos_x) > 0)):
        return False, 0.0

      lookahead = float(np.clip(v_ego, _LOOKAHEAD_MIN_M, _LOOKAHEAD_MAX_M))
      model_y = float(np.interp(lookahead, pos_x, pos_y))
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0

    # Laneline contribution, blended in by confidence. scale=0 (model-position-only, i.e. just
    # the user's offset bias) whenever lines are missing, low-confidence, or structurally bad --
    # see _laneline_blend for the confidence formula (ported from lateral_curv_ext.py).
    scale, laneline_center_y = self._laneline_blend(model, lookahead)
    target_y = model_y * (1.0 - scale) + laneline_center_y * scale

    error = (target_y + offset) - model_y
    raw = 2.0 * error / (lookahead ** 2)
    lower, upper = self._boundary_correction_limits(model, pos_x, pos_y, lookahead)
    return True, float(np.clip(raw, lower, upper))

  def _boundary_correction_limits(self, model, pos_x, pos_y, lookahead: float) -> tuple[float, float]:
    """Limit only added trim: zero always remains feasible, including in a too-narrow lane.

    Use the same small-curvature geometry as the trim (delta_y = 0.5 * delta_kappa * x**2).
    Check multiple distances so a safe-looking endpoint cannot hide a close boundary earlier
    in a curve. Only interpolate inside both curves' support; never extrapolate a road edge.
    """
    lower, upper = -_MAX_RAW_CORRECTION, _MAX_RAW_CORRECTION
    start = max(_BOUNDARY_NEAR_M, float(pos_x[0]))
    end = min(lookahead, float(pos_x[-1]))
    if end < start:
      return lower, upper
    samples = np.linspace(start, end, _BOUNDARY_SAMPLES)
    baseline = np.interp(samples, pos_x, pos_y)
    # Convert normal-to-path clearance into y clearance at the same x on curves.
    slope = np.interp(samples, pos_x, np.gradient(pos_y, pos_x))
    normal_scale = np.hypot(1.0, slope)

    for curve_name, std_name, indices in (("laneLines", "laneLineStds", (1, 2)),
                                          ("roadEdges", "roadEdgeStds", (0, 1))):
      for side, index in enumerate(indices):
        try:
          curve = getattr(model, curve_name)[index]
          std = float(getattr(model, std_name)[index])
          if not np.isfinite(std) or not 0.0 <= std <= _BOUNDARY_MAX_STD_M:
            continue
          if curve_name == "laneLines":
            prob = float(model.laneLineProbs[index])
            if not np.isfinite(prob) or not _BOUNDARY_MIN_PROB <= prob <= 1.0:
              continue
          boundary_x = np.asarray(curve.x, dtype=float)
          boundary_y = np.asarray(curve.y, dtype=float)
          if (boundary_x.ndim != 1 or boundary_y.ndim != 1 or boundary_x.size < 2 or
              boundary_x.size != boundary_y.size or not np.isfinite(boundary_x).all() or
              not np.isfinite(boundary_y).all() or not np.all(np.diff(boundary_x) > 0)):
            continue
          covered = (samples >= boundary_x[0]) & (samples <= boundary_x[-1])
          if not covered.any():
            continue
          boundary = np.interp(samples[covered], boundary_x, boundary_y)
          clearance = (_VEHICLE_HALF_WIDTH_AND_MARGIN_M + std) * normal_scale[covered]
          limit = 2.0 * (boundary + (clearance if side == 0 else -clearance) - baseline[covered]) / samples[covered] ** 2
          if side == 0:
            lower = max(lower, min(0.0, float(np.max(limit))))
          else:
            upper = min(upper, max(0.0, float(np.min(limit))))
        except (AttributeError, IndexError, TypeError, ValueError):
          continue
    return lower, upper

  def _laneline_blend(self, model, lookahead: float) -> tuple[float, float]:
    """Returns (scale, laneline_center_y). scale=0 whenever lanelines can't be trusted (missing,
    low-probability, structurally invalid) -- center_y is unused/meaningless in that case since
    it's weighted out by scale in the caller's blend."""
    try:
      lane_lines = model.laneLines
      probs = model.laneLineProbs
      stds = model.laneLineStds # a line could exist but this verifies how sure it is of where it is
      if len(lane_lines) < 3 or len(probs) < 3 or len(stds) < 3:
        return 0.0, 0.0
      if not all(np.isfinite(probs[i]) and 0.0 <= probs[i] <= 1.0 and
                 np.isfinite(stds[i]) and stds[i] >= 0.0 for i in (1, 2)):
        return 0.0, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      if (left_x.size < 2 or left_x.size != left_y.size or
          right_x.size < 2 or right_x.size != right_y.size):
        return 0.0, 0.0
      if not (np.isfinite(left_x).all() and np.isfinite(left_y).all() and
              np.isfinite(right_x).all() and np.isfinite(right_y).all()):
        return 0.0, 0.0
      if not (np.all(np.diff(left_x) > 0) and np.all(np.diff(right_x) > 0)):
        return 0.0, 0.0

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left

      # Same confidence formula as lateral_curv_ext.py's path_offset blend: width-tolerance
      # (penalizes implausibly wide/merging-looking detections) combined with per-line
      # probability via min() -- a single missing/unreliable line (e.g. no line on the curb
      # side, only a center stripe) drags confidence toward 0 on its own.
      width_tolerance = float(np.interp(width, _WIDTH_TOLERANCE_BP, _WIDTH_TOLERANCE_V))
      std_tolerance = float(np.interp(max(float(stds[1]), float(stds[2])), _STD_TOLERANCE_BP, _STD_TOLERANCE_V)) #StarPilot stopped at 0.3, this fades the std through the table
      confidence = min(float(probs[1]), float(probs[2]), width_tolerance, std_tolerance) #confidence is the weakest signal
      scale = float(np.clip(np.interp(confidence, _CONFIDENCE_BP, _CONFIDENCE_V), 0.0, 1.0))
      center_y = 0.5 * (left + right)
      return scale, center_y
    except (AttributeError, IndexError, TypeError, ValueError):
      return 0.0, 0.0
