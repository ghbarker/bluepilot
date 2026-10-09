"""BluePilot: lifecycle controller for the Ford angle-mode auto-calibration.

AutoCalPipeline (angle_autocal.py) is pure math with no I/O. This controller owns
everything between that math and the car: arm/disarm from the toggle, evidence
persistence to FordAngleAutoCalState, nudge writes to the factor params, errors to
FordAngleAutoCalError, and the telemetry status string. FordLateralAngleExt calls
poll_params() at ~1 Hz, feed() per 20 Hz lateral frame, and idle() when inactive.

Nudges are written straight to the factor params (blocking); the strategy reads them
back through poll_params, so the live steering factors have a single owner and no
in-memory adopt path is needed.
"""
import json
import math
import time

from opendbc.sunnypilot.car.ford.angle_autocal import AutoCalPipeline, Frame

SAVE_PERIOD_S = 30.0
# Half the menu granularity (0.01): a larger change not written by the nudger
# is treated as a driver hand-edit.
EDIT_TOL = 0.005
FACTOR_KEYS = ('FordLowSpeedFactor_ang', 'FordHighSpeedFactor_ang')


def _state_locked(state: str) -> bool:
  """True when the persisted state says the calibration is finished.
  Legacy pre-JSON states ("done low=... high=... verified") stay honored."""
  if state.startswith("done"):
    return True
  if state.startswith("{"):
    try:
      d = json.loads(state)
      pipe = d.get("pipe", {})
      pending = any(any(pipe.get(key, [])) for key in ("verify", "verify_hold", "recovery"))
      return d.get("phase") == "locked" and not pending
    except (ValueError, AttributeError, TypeError):
      return False
  return False


def _restore(pipeline, state: str):
  """Load serialized evidence into a fresh pipeline; anything unparseable (legacy round
  strings, garbage, empty) simply starts a fresh collection."""
  if not state.startswith("{"):
    return
  try:
    d = json.loads(state)
    pipe = d.get("pipe")
    if isinstance(pipe, dict) and int(d.get("v", 0)) == 1:
      pipeline.from_dict(pipe)
  except (ValueError, KeyError, TypeError):
    pass


class AutoCalController:
  def __init__(self, dt: float):
    self.dt = dt
    self.enabled = False
    self.done = True            # conservative until params are read
    self.pipeline = None        # AutoCalPipeline while collecting
    self.status = ""            # live ground-truth status, published in telemetry
    self._pause_reason = "delay"  # BluePilot: display only; never used for admission
    self._last_response_time = None
    self._params = None
    self._last_written = None   # (low, high) the nudger last wrote; a different param value is a user edit
    self._save_s = 0.0
    self._dirty = False

  # -- ~1 Hz: toggle, restore, user edits, status ------------------------------------------
  def poll_params(self, params, low_factor: float, high_factor: float, platform_gain_high: float):
    """Arm/disarm from the toggle, restore evidence on arm, detect user hand-edits of the
    factor params, refresh the status string. low/high are the currently applied values."""
    try:
      if params.get_bool("FordAngleAutoCalReset"):
        # Erase calibration memory: evidence, error channel, lock and factors all go
        # back to neutral so the car steers stock immediately and collection restarts.
        # Idempotent with the UI's own param clears; covers non-UI writers too.
        params.put_bool("FordAngleAutoCalReset", False)
        params.put("FordAngleAutoCalState", "", True)
        params.put("FordAngleAutoCalError", "")
        params.put("FordLowSpeedFactor_ang", 1.0)
        params.put("FordHighSpeedFactor_ang", 1.0)
        self.pipeline = None
        self.done = False
        self._last_written = (1.0, 1.0)
        self._dirty = False
        self._params = params
        self.status = "reset"
        return
      enabled = bool(params.get_bool("FordAngleAutoCal"))
      if not enabled:
        self._disarm(params)
        return
      # Lock behavior toggle (default ON): with the lock OFF the calibration never
      # freezes — and an EXISTING lock is treated as "resume from this evidence", not
      # as finished, so flipping the toggle un-locks without losing anything.
      # Chestnut's get_bool does not return the registered default for an absent key.
      lock_value = params.get("FordAngleAutoCalLock", return_default=True)
      lock_on = True if lock_value is None else bool(lock_value)
      state = params.get("FordAngleAutoCalState", return_default=True) or ""
      if isinstance(state, bytes):
        state = state.decode("utf-8", errors="replace")
      if self.pipeline is None:
        self.done = _state_locked(state) and lock_on
      else:
        self.pipeline.lock_enabled = lock_on
        if not lock_on and self.pipeline.locked:
          self.pipeline.locked = False
          self.pipeline.stable_s = 0.0
        self.done = self.pipeline.locked
      self.enabled = enabled and not self.done
      if self.enabled and self.pipeline is None:
        # Arm: build the pipeline, restore prior-drive evidence, baseline the nudger on
        # the currently applied factors.
        self.pipeline = AutoCalPipeline(platform_gain_high, dt=self.dt)
        _restore(self.pipeline, state)
        # A manual edit made while stopped is already present at our first poll.
        # It must cancel an old trial/recovery, not wait forever for its old target.
        for half, current in enumerate((low_factor, high_factor)):
          pending = self.pipeline.recovery[half] or self.pipeline.verify[half]
          if pending is not None and not any(abs(current - pending[key]) < 1e-6 for key in ('frm', 'to')):
            self.pipeline.user_edit()
            break
        self.pipeline.lock_enabled = lock_on
        if not lock_on and self.pipeline.locked:
          self.pipeline.locked = False  # resuming a previously locked calibration
          self.pipeline.stable_s = 0.0
        # BluePilot: persisted responses cannot establish readiness in a new process.
        self.pause_for_delay()
        self._last_written = (float(low_factor), float(high_factor))
      elif not self.enabled:
        self.pipeline = None
      else:
        # User hand-edit: a factor param differs from what the nudger last wrote. The
        # nudger records each blocking write separately, including partial failures.
        # A mismatch with that record is a driver edit. Adopt their value
        # (already live in the strategy) and soft-reset confidence; evidence is not wiped.
        lw = self._last_written
        moved = lw is not None and (abs(low_factor - lw[0]) > EDIT_TOL
                                    or abs(high_factor - lw[1]) > EDIT_TOL)
        if moved:
          self.pipeline.user_edit()
          self._last_written = (float(low_factor), float(high_factor))
          self._dirty = True
      self._params = params
      # Live status for telemetry: published from actual controller state (ground truth),
      # never from a param re-read — a param/telemetry mismatch is exactly the failure
      # mode that made earlier on-device issues undiagnosable.
      if self.done:
        self.status = "locked"
      elif not self.enabled:
        self.status = "off"
      else:
        # Armed: compact JSON so live dashboards (phone /lateral cards) can render the
        # per-anchor story — evidence progress, measured response, proposed step, and
        # the adjust-then-verify judgment — from the same ground truth the nudger uses.
        ui = self.pipeline.ui_state(low_factor, high_factor)
        ui["n"] = self.pipeline.est.n
        if self._pause_reason:
          ui["pause"] = self._pause_reason
        self.status = json.dumps(ui, separators=(",", ":"))
    except Exception as e:
      self.enabled = False
      self.status = f"tick error: {type(e).__name__}: {e}"[:200]
      self._error(self.status)

  # -- 20 Hz frames ------------------------------------------------------------------------
  def _disarm(self, params):
    """BluePilot: own off/reset persistence so UI and remote toggles behave alike."""
    self.enabled = False
    self.pipeline = None
    self.done = False
    self._dirty = False
    self._save_s = 0.0
    self._last_written = None
    self._params = params
    self.status = "off"
    if params.get("FordAngleAutoCalState", return_default=True):
      params.put("FordAngleAutoCalState", "", True)

  def idle(self):
    """Frames where lateral is inactive (disengaged / human turn / stall blip)."""
    self._pause_reason = "inactive"
    if self.pipeline is not None:
      self.pipeline.idle(elapsed_s=self._response_elapsed())

  def _response_elapsed(self):
    now = time.monotonic()
    elapsed = self.dt if self._last_response_time is None else max(0.0, now - self._last_response_time)
    self._last_response_time = now
    return elapsed

  def pause_for_delay(self):
    """Pause collection, writes and lock progress until liveDelay is ready again."""
    self._pause_reason = "delay"
    self._last_response_time = None
    if self.pipeline is not None:
      self.pipeline.pause_for_delay()
      self._dirty = True

  def pause_for_steering_feedback(self):
    """Do not learn, propose or write gains without unrestricted EPS feedback."""
    self.idle()
    self._pause_reason = "steering_feedback"
    if self.pipeline is not None:
      self.pipeline.stable_s = 0.0
      self._dirty = True

  def feed(self, frame: Frame, delay_estimated: bool):
    """One active lateral frame. Nudges are written to the factor params (the strategy
    reads them back — single reader); the lock -> disarm transition and save cadence
    happen here."""
    if not self.enabled or self.pipeline is None:
      return
    # The settings callback can clear persisted evidence before the 1 Hz poll.
    # Observe disable before another frame can nudge or save the old pipeline back.
    if self._params is not None and not self._params.get_bool("FordAngleAutoCal"):
      self._disarm(self._params)
      return
    if not delay_estimated or not math.isfinite(frame.lateral_delay) or frame.lateral_delay <= 0.0:
      # BluePilot: learning or unusable delay cannot align a measured response with
      # its command. Do not recommend even a pending rollback until delay is ready.
      self.pause_for_delay()
      return
    self._pause_reason = ""
    committed = self.pipeline.update(frame, elapsed_s=self._response_elapsed())
    if committed:
      self._dirty = True
    applied = (frame.low_factor, frame.high_factor)
    rec = self.pipeline.recommend(frame.low_factor, frame.high_factor)
    if rec is not None and self._apply_nudge(rec, applied):
      applied = rec
    if self.pipeline is None:
      return  # disable was observed while the checkpoint was being written
    if self.pipeline.locked:
      self._save("locked", applied)
      self.done = True
      self.enabled = False
      self.pipeline = None
    else:
      self._save_s += self.dt
      if self._dirty and self._save_s >= SAVE_PERIOD_S:
        self._save("collecting", applied)

  # -- params I/O --------------------------------------------------------------------------
  def _read_factors(self):
    values = tuple(float(self._params.get(key, return_default=True)) for key in FACTOR_KEYS)
    if not all(math.isfinite(value) for value in values):
      raise ValueError('nonfinite adjustment factor')
    return values

  def _apply_nudge(self, rec, applied=None) -> bool:
    """Save the pending trial/recovery before any factor changes, then read back writes.

    Each observed write owns its value even if a later write fails. A crash after
    either factor lands can restore the saved bounded rollback. The normal strategy
    reader still has to observe the factors before responses can verify a trial.
    """
    if self._params is None or self.pipeline is None:
      return False
    if not self._params.get_bool('FordAngleAutoCal'):
      self._disarm(self._params)
      return False
    before = self._last_written if applied is None else applied
    try:
      actual = self._read_factors()
      if before is None or any(abs(a - b) > 1e-6 for a, b in zip(actual, before, strict=True)):
        self.pipeline.user_edit()
        self._last_written = actual
        self._dirty = True
        return False  # the driver changed a factor after this frame was constructed
      self._last_written = actual
      # All state checkpoints are blocking: an older queued save must not overwrite
      # this write-ahead record after the corresponding factor has already changed.
      if not self._save('collecting', actual):
        return False
      for half, key in enumerate(FACTOR_KEYS):
        if not self._params.get_bool('FordAngleAutoCal'):
          self._disarm(self._params)
          return False
        observed = self._read_factors()
        if any(abs(a - b) > 1e-6 for a, b in zip(observed, self._last_written, strict=True)):
          self.pipeline.user_edit()
          self._last_written = observed
          self._dirty = True
          return False  # a manual edit arrived during the checkpoint or previous write
        if float(rec[half]) == actual[half]:
          continue
        self._params.put(key, float(rec[half]), True)
        observed = self._read_factors()
        if abs(observed[half] - rec[half]) > 1e-6:
          raise OSError(f'{key} write did not read back')
        written = list(self._last_written)
        written[half] = observed[half]
        self._last_written = tuple(written)
    except Exception as e:
      # Some failures are reported after a rename. Account only for old/intended
      # values; unexpected values remain visible as manual edits at the next poll.
      try:
        observed = self._read_factors()
        written = list(self._last_written or before)
        for half in (0, 1):
          if any(abs(observed[half] - value) < 1e-6 for value in (before[half], rec[half])):
            written[half] = observed[half]
        self._last_written = tuple(written)
      except Exception:
        pass
      self._dirty = True
      self._error(f"nudge write failed: {type(e).__name__}: {e}")
      return False
    # The pre-write record already protects restart recovery. The periodic
    # checkpoint will record the observed new pair; avoid another disk sync here.
    self._dirty = True
    return True

  def _error(self, msg: str):
    """Park diagnostics in their OWN param, never FordAngleAutoCalState: an error written
    just before ignition-off must not be able to overwrite the serialized evidence."""
    try:
      if self._params is not None:
        self._params.put("FordAngleAutoCalError", f"{msg[:300]}")
    except Exception:
      pass

  def _save(self, phase: str, applied):
    """Ordered checkpoint; return whether a blocking save also read back correctly."""
    if self._params is None or self.pipeline is None:
      return False
    d = {
      "v": 1,
      "phase": phase,
      "pipe": self.pipeline.to_dict(),
      "applied": {"low": round(applied[0], 2), "high": round(applied[1], 2)},
    }
    sol = self.pipeline.est.solve()
    if sol is not None:
      low_t, high_t, st = sol
      d["target"] = {"low": round(low_t, 2), "high": round(high_t, 2)}
      d["weight"] = {"low": round(st["weight_low"], 1), "high": round(st["weight_high"], 1)}
      d["stderr"] = {"low": round(st["stderr_eff_low"], 3), "high": round(st["stderr_eff_high"], 3)}
      d["stable_s"] = round(self.pipeline.stable_s, 1)
    try:
      state = json.dumps(d, separators=(",", ":"), allow_nan=False)
      self._params.put("FordAngleAutoCalState", state, True)
      stored = self._params.get('FordAngleAutoCalState', return_default=True)
      if isinstance(stored, bytes):
        stored = stored.decode('utf-8')
      if stored != state:
        raise OSError('calibration state write did not read back')
    except Exception as e:
      self._dirty = True
      self._error(f"state save failed: {type(e).__name__}: {e}")
      return False
    self._save_s = 0.0
    self._dirty = False
    return True
