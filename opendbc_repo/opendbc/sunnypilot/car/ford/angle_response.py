"""BluePilot: bounded, qualified response evidence for calibration trials.

Only the existing admission pipeline can add observations. Unlike the display's
EMA, evidence weight does not decay between clean turns. Age and storage are
bounded, and before/after comparisons use the same speed/direction mix.
"""
from collections import deque
import math


RESPONSE_MAX_AGE_S = 600.0
RESPONSE_MAX_WEIGHT = 24.0  # twice the largest requalification requirement


def response_stats(totals):
  w, wr, wr2 = totals
  if w <= 1e-6:
    return 0.0, None, math.inf
  mean = wr / w
  stderr = math.sqrt(max(0.0, wr2 / w - mean * mean) / max(1.0, w - 1.0))
  return w, mean, stderr


class ResponseWindow:
  def __init__(self):
    self.clock = 0.0
    self.rows = {0: deque(), 1: deque()}
    self.bins = {0: {}, 1: {}}
    self.weight = {0: 0.0, 1: 0.0}

  def clear(self):
    for half in (0, 1):
      self.rows[half].clear()
      self.bins[half].clear()
      self.weight[half] = 0.0

  def _remove(self, half):
    _, key, w, wr, wr2 = self.rows[half].popleft()
    totals = self.bins[half][key]
    for i, value in enumerate((w, wr, wr2)):
      totals[i] = max(0.0, totals[i] - value)
    self.weight[half] = max(0.0, self.weight[half] - w)

  def advance(self, seconds):
    if not math.isfinite(seconds) or seconds < 0.0:
      self.clear()
      self.clock += RESPONSE_MAX_AGE_S  # invalidate in-flight baselines too
      return
    self.clock += seconds
    for half in (0, 1):
      while self.rows[half] and self.clock - self.rows[half][0][0] >= RESPONSE_MAX_AGE_S:
        self._remove(half)

  def add(self, half, key, weight, ratio):
    totals = self.bins[half].setdefault(key, [0.0, 0.0, 0.0])
    values = (weight, weight * ratio, weight * ratio * ratio)
    self.rows[half].append((self.clock, key, *values))
    self.weight[half] += weight
    for i, value in enumerate(values):
      totals[i] += value
    while self.rows[half] and self.weight[half] > RESPONSE_MAX_WEIGHT + 1e-6:
      self._remove(half)

  def response(self, half):
    return response_stats([sum(t[i] for t in self.bins[half].values()) for i in range(3)])

  def snapshot(self, half):
    return {"time": self.rows[half][0][0] if self.rows[half] else self.clock,
            "bins": {str(k): list(v) for k, v in self.bins[half].items() if v[0] > 1e-6}}

  def comparable(self, half, baseline):
    """Equal weight for each shared speed/direction bin on both sides of a trial."""
    before = [0.0, 0.0, 0.0]
    after = [0.0, 0.0, 0.0]
    for key, old in baseline["bins"].items():
      new = self.bins[half].get(int(key))
      if new is None or min(old[0], new[0]) <= 1e-6:
        continue
      w = min(old[0], new[0])
      for i in range(3):
        before[i] += w * old[i] / old[0]
        after[i] += w * new[i] / new[0]
    w, pre, pre_stderr = response_stats(before)
    _, post, post_stderr = response_stats(after)
    return w, pre, post, pre_stderr, post_stderr
