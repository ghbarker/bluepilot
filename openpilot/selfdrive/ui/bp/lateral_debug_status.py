"""Read-only, fresh steering diagnostics for the small-screen debug panel."""
import json
import math

from openpilot.selfdrive.ui.bp.angle_autocal_status import calibration_status_text


def _fresh(sm, service: str, now_ns: int):
  try:
    if sm.valid.get(service, False) and sm.alive.get(service, False):
      age = now_ns - sm.logMonoTime[service]
      if 0 <= age <= 2_000_000_000:
        return sm[service]
  except (KeyError, AttributeError, TypeError, ValueError):
    pass
  return None


def _positive(value):
  try:
    value = float(value)
    return value if math.isfinite(value) and value > 0 else None
  except (TypeError, ValueError, OverflowError):
    return None


def diagnostic_lines(sm, now_ns: int, onroad: bool, default_delay=None) -> tuple[str, str]:
  """No socket creation or Params reads; consume the UI's existing subscriptions."""
  delay = _fresh(sm, 'lateralDelay', now_ns) if onroad else None
  delay_text = 'Delay unavailable'
  if delay is not None:
    value = _positive(getattr(delay, 'lateralDelay', None))
    status = str(getattr(delay, 'status', 'invalid'))
    if status == 'estimated' and value is not None:
      delay_text = f'Delay {value:.3f}s live'
    elif status == 'unestimated':
      delay_text = 'Delay learning'
    else:
      delay_text = 'Delay invalid'
  else:
    value = _positive(default_delay)
    if value is not None:
      delay_text = f'Delay {value:.3f}s default'

  controller = _fresh(sm, 'controllerStateBP', now_ns) if onroad else None
  if controller is None:
    return delay_text, 'Auto-cal: unavailable' if onroad else 'Auto-cal: waiting for drive'
  enabled = bool(getattr(controller, 'bmsAngleAutoCalibrate', False))
  raw = getattr(controller, 'bmsAngleAutoCalState', None)
  if not isinstance(raw, str):
    raw = None
  try:
    summary = calibration_status_text(enabled, raw, onroad=True)
  except (TypeError, ValueError, AttributeError, OverflowError):
    summary = 'Status unavailable'
  line = f'Auto-cal: {summary}'
  # Evidence is allowed to decay; it is not a monotonic completion percentage.
  # Keep the existing phase/pause/rollback summary instead of implying completion.
  if enabled and raw and raw.startswith('{'):
    try:
      state = json.loads(raw)
      if isinstance(state, dict) and not state.get('pause') and summary.startswith(('Collecting', 'Low-speed:', 'High-speed:')):
        evidence = []
        for name in ('low', 'high'):
          band = state.get(name)
          if not isinstance(band, dict):
            break
          weight = float(band.get('w', 0))
          need = _positive(band.get('need'))
          if not math.isfinite(weight) or weight < 0 or need is None:
            break
          evidence.append(f'{name.title()} {min(weight, need):.1f}/{need:.1f}')
        if len(evidence) == 2:
          line += ' | ' + '  '.join(evidence)
    except (ValueError, TypeError, OverflowError):
      pass
  return delay_text, line
