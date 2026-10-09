"""Offline unwind experiment with finite, planner-progress-based restoration.

Never imported by the driving controller. K=1 is an UNVALIDATED assumption:
one curvature unit of net planner entry restores one unit of withheld optional
prediction. It is not a vehicle-response model or a safety qualification.

This addresses the proportional taper's persistent old peak/direction without
a restore timer. Growing raw prediction can still overpower the taper or oppose
the planner after restoration; blocking acceptance tests document both cases.
"""
from dataclasses import dataclass
import math


@dataclass
class PlannerUnwindRecovery:
  restore_gain: float = 1.0
  direction: float = 0.0
  anchor_desired: float = 0.0
  anchor_withheld: float = 0.0
  last_desired: float = 0.0
  budget: float = 0.0
  scale: float = 1.0
  phase: str = 'unwind'
  recovery_origin: float = 0.0
  recovery_budget: float = 0.0
  crossing: bool = False

  def __post_init__(self):
    if not math.isfinite(self.restore_gain) or self.restore_gain <= 0.0:
      raise ValueError('restore_gain must be positive and finite')

  def reset(self):
    self.direction = 0.0
    self.anchor_desired = 0.0
    self.anchor_withheld = 0.0
    self.last_desired = 0.0
    self.budget = 0.0
    self.scale = 1.0
    self.phase = 'unwind'
    self.recovery_origin = 0.0
    self.recovery_budget = 0.0
    self.crossing = False

  def _reanchor(self, desired: float):
    self.reset()
    self.direction = math.copysign(1.0, desired) if desired != 0.0 else 0.0
    self.anchor_desired = abs(desired)
    self.last_desired = desired

  def _unwind_budget(self, signed_desired: float, retaining: float) -> float:
    # The saved absolute budget is carried through an interrupted recovery.
    # At the anchor, this equals that budget even if prediction has changed;
    # at zero, every current retaining contribution is withheld.
    progress = 1.0 - min(1.0, max(0.0, signed_desired / self.anchor_desired))
    return self.anchor_withheld + max(0.0, retaining - self.anchor_withheld) * progress

  def _start_crossing(self, desired: float, retaining: float):
    self.phase = 'recovery'
    self.crossing = True
    self.recovery_origin = 0.0
    self.recovery_budget = max(self.budget, retaining)
    self.budget = max(0.0, self.recovery_budget - self.restore_gain * abs(desired))

  def update(self, desired: float, nominal: float) -> float:
    """Return an experimental pre-trim request; callers own lifecycle resets.

    The correction remains between the planner and nominal prediction blend.
    Existing downstream constraints must still run. This function does not
    guarantee monotonic unwind, command direction, or a safe vehicle trajectory.
    """
    if not math.isfinite(desired) or not math.isfinite(nominal):
      self.reset()
      raise ValueError('unwind candidate requires finite curvature inputs')

    if self.direction == 0.0:
      self._reanchor(desired)
      return nominal

    direction = self.direction
    signed_desired = direction * desired
    previous_desired = direction * self.last_desired
    retaining = max(0.0, direction * (nominal - desired))

    if self.phase == 'recovery':
      if self.crossing:
        # Zero-crossing oscillations earn no cumulative restoration: the
        # budget depends on current distance from zero, not traveled distance.
        self.budget = max(0.0, self.recovery_budget - self.restore_gain * abs(desired))
      elif signed_desired >= self.recovery_origin:
        progress = signed_desired - self.recovery_origin
        self.budget = max(0.0, self.recovery_budget - self.restore_gain * progress)
      elif signed_desired > 0.0:
        # A genuinely lower trough continues the unwind from the OLD recovery
        # anchor. Do not replace the budget with a new proportional target.
        self.phase = 'unwind'
        self.anchor_desired = self.recovery_origin
        self.anchor_withheld = self.recovery_budget
        self.budget = self._unwind_budget(signed_desired, retaining)
      else:
        self._start_crossing(desired, retaining)
    elif signed_desired <= 0.0:
      self._start_crossing(desired, retaining)
    elif signed_desired > previous_desired:
      self.phase = 'recovery'
      self.crossing = False
      self.recovery_origin = previous_desired
      self.recovery_budget = self.budget
      self.budget = max(0.0, self.recovery_budget - self.restore_gain * (signed_desired - previous_desired))
    else:
      self.budget = self._unwind_budget(signed_desired, retaining)

    if self.phase == 'recovery' and self.budget == 0.0:
      # The remaining restriction is already zero; reanchoring cannot change
      # this output or inject a one-tick restoration step.
      self._reanchor(desired)
      return nominal

    withheld = min(retaining, self.budget)
    self.scale = 1.0 - withheld / retaining if retaining > 0.0 else 1.0
    self.last_desired = desired
    return nominal - direction * withheld
