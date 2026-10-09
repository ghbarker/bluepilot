"""Offline Ford angle unwind candidate. Never imported by the driving controller.

The planner's decreasing request is itself a continuous measure of unwind; no
fixed per-frame drop threshold or raw-prediction 'entering' veto is used. Taper
only the additive prediction contribution that retains the preceding turn.

This is an experiment, not a validated controller. In particular, a growing raw
contribution can still outweigh the taper, and an S-bend can retain the previous
direction longer than intended. The replay and acceptance tests expose those
limitations rather than enabling this behavior on a vehicle.
"""
from dataclasses import dataclass
import math


@dataclass
class PlannerUnwindTaper:
  peak: float = 0.0
  scale: float = 1.0

  def reset(self):
    self.peak = 0.0
    self.scale = 1.0

  def update(self, desired: float, nominal: float) -> float:
    """Return a candidate pre-trim request from finite curvature inputs.

    Nominal is the existing planner/prediction blend, before lane trim and all
    existing constraints. No limit, gain, lookahead, or safety state is changed.
    Lifecycle/model validity must be handled by the caller via reset().
    """
    if not math.isfinite(desired) or not math.isfinite(nominal):
      self.reset()
      raise ValueError("unwind candidate requires finite curvature inputs")

    if self.peak == 0.0:
      self.peak = desired
    if self.peak == 0.0:
      # No established turn: do not invent an unwind episode at startup.
      self.scale = 1.0
      return nominal

    direction = math.copysign(1.0, self.peak)
    if direction * desired >= abs(self.peak):
      self.peak = desired
    contribution = nominal - desired
    self.scale = min(1.0, max(0.0, desired / self.peak))
    retaining = direction * contribution > 0.0
    result = desired + contribution * self.scale if retaining else nominal

    # Reset across zero only when doing so leaves this output unchanged. This
    # prevents an immediate restoration step, but does not prove S-bend liveness.
    if direction * desired <= 0.0 and not retaining:
      self.peak = desired
      self.scale = 1.0
    return result


def instrument_source(source: str, candidate: str = 'proportional') -> str:
  """Install the experiment in an in-memory controller used by replay only.

  Fail on source drift. Production files are never rewritten. All downstream
  constraints execute unmodified; lifecycle exits and unhealthy models reset
  the experiment so evidence cannot cross inactive/override/dropout intervals.
  """
  init = '    self._desired_curvature_last = 0.0'
  blend = '    requested_curvature = predicted_curvature * b_blend + desired_curvature * (1.0 - b_blend)'
  reset = '      self.lane_center_trim.reset()'
  if source.count(init) != 1 or source.count(blend) != 1 or source.count(reset) != 3:
    raise ValueError('controller source changed; review offline instrument points')
  imports = {
    'proportional': 'from tools.bluepilot_chestnut.experiments.ford_unwind import PlannerUnwindTaper\n',
    'net-progress': 'from tools.bluepilot_chestnut.experiments.ford_unwind_recovery import PlannerUnwindRecovery as PlannerUnwindTaper\n',
  }
  source = imports[candidate] + source
  source = source.replace(init, init + '\n    self.unwind_candidate = PlannerUnwindTaper()')
  source = source.replace(reset, reset + '\n      self.unwind_candidate.reset()')
  return source.replace(blend, blend + '''
    nominal_request = requested_curvature
    if model is None:
      self.unwind_candidate.reset()
    else:
      requested_curvature = self.unwind_candidate.update(desired_curvature, nominal_request)
    unwind_scale = self.unwind_candidate.scale
''')
