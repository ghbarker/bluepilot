"""Read-only calibration summaries. Saved evidence must never look like live status."""
import json


def _object(raw: str) -> dict:
  try:
    value = json.loads(raw)
    return value if isinstance(value, dict) else {}
  except (ValueError, TypeError):
    return {}


def calibration_status_text(enabled: bool, live: str | None, saved: str = "", *, onroad: bool = False) -> str:
  if not enabled:
    return "Off"
  if live is None:
    if onroad:
      return "Status unavailable"
    state = _object(saved)
    pipe = state.get("pipe", {})
    if not isinstance(pipe, dict):
      return "Saved status unavailable"
    pending = any(isinstance(pipe.get(k), list) and any(pipe[k]) for k in ("verify", "verify_hold", "recovery"))
    if (state.get("phase") == "locked" and not pending) or saved.startswith("done"):
      return "Saved: locked"
    if isinstance(pipe.get("recovery"), list) and any(pipe["recovery"]):
      return "Saved: rollback pending"
    if isinstance(pipe.get("verify"), list) and any(pipe["verify"]):
      return "Saved: adjustment under review"
    if state.get("phase") == "collecting":
      return "Saved: collecting"
    return "Waiting for drive"
  if live == "locked":
    return "Locked"
  if live == "reset":
    return "Resetting"
  if live == "off":
    return "Waiting for controller"
  if live.startswith("tick error:"):
    return "Calibration error"
  state = _object(live)
  if state.get("pause") == "delay":
    return "Waiting for steering delay"
  if state.get("pause") == "steering_feedback":
    return "Waiting for clear steering feedback"
  if state.get("pause") == "inactive":
    return "Waiting for active steering"
  low, high = state.get("low"), state.get("high")
  if not isinstance(low, dict) or not isinstance(high, dict):
    return "Status unavailable"
  if any(d.get("rollback") for d in (low, high)):
    return "Reverting adjustment"
  phases = [d.get("ph") for d in (low, high)]
  if any(phase not in ("collect", "propose", "verify", "good") for phase in phases):
    return "Status unavailable"
  if "verify" in phases:
    if any(d.get("ph") == "verify" and d.get("reason") == "matching_turns" for d in (low, high)):
      return "Testing: matching turns needed"
    if any(d.get("ph") == "verify" and d.get("reason") == "response_consistency" for d in (low, high)):
      return "Testing: response inconsistent"
    return "Testing adjustment"
  if "propose" in phases:
    return "Checking next adjustment"
  if phases == ["good", "good"]:
    if any(d.get("reason") == "lock_evidence" for d in (low, high)):
      return "Factors matched; confirming"
    return "Response matched; monitoring"
  focus = state.get("active")
  choices = [("low", low), ("high", high)]
  if focus == "high":
    choices.reverse()
  reasons = {
    "fit_evidence": "more turn data needed",
    "fit_confidence": "response inconsistent",
    "fresh_evidence": "collecting clean turns",
    "response_consistency": "response inconsistent",
    "speed_range": "waiting for speed range",
    "clean_frame": "waiting for clean data",
    "settling": "waiting between trials",
  }
  for name, detail in choices:
    if reason := reasons.get(detail.get("reason")):
      return f"{name.capitalize()}-speed: {reason}"
  if any(d.get("vr") in ("failed", "expired") for d in (low, high)):
    return "Collecting after rollback"
  return "Collecting response data"
