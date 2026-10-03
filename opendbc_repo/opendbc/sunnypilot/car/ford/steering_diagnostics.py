"""Read-only Ford diagnostics. These values never feed steering or warning decisions."""
import math
import time


EPS_MAX_AGE_NS = 100_000_000


def can_clock_nanos():
  """Match C++ CAN event CLOCK_BOOTTIME, including time spent suspended on Linux."""
  if hasattr(time, "CLOCK_BOOTTIME"):
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  # Non-Linux source-test hosts do not expose BOOTTIME; C++ uses MONOTONIC on macOS.
  return time.monotonic_ns()


def steering_command_snapshot(message, frame, now_nanos):
  """Decode the final packed request, including its wire sign and quantization.

  This records a request queued for transmission, not Panda/EPS acceptance. Keep
  the returned snapshot and its timestamp unchanged between 20 Hz sends.
  """
  unavailable = {"dataAvailable": False}
  try:
    address, data, _bus = message
    if address not in (0x3D3, 0x3D6) or not isinstance(data, (bytes, bytearray)) or len(data) != 8:
      return unavailable
    if not isinstance(frame, int) or not 0 <= frame < 2**64 or not isinstance(now_nanos, int) or not 0 < now_nanos < 2**64:
      return unavailable
    can_fd = address == 0x3D6
    if can_fd:
      mode = (data[0] >> 4) & 7
      angle_raw = ((data[3] & 0x1F) << 6) | (data[4] >> 2)
      curvature_raw = (data[2] << 3) | (data[3] >> 5)
    else:
      mode = (data[4] >> 2) & 7
      angle_raw = (data[3] << 3) | (data[4] >> 5)
      curvature_raw = (data[0] << 3) | (data[1] >> 5)
    return {
      "dataAvailable": True,
      "pathAngle": (angle_raw - 1000) * 0.0005,
      "curvature": (curvature_raw - 1000) * 0.00002,
      "mode": mode,
      "canFd": can_fd,
      "frame": frame,
      "sourceMonoTime": now_nanos,
    }
  except (TypeError, ValueError, OverflowError, IndexError):
    return unavailable


def fill_eps_diagnostics(feedback, cp, now_nanos):
  """EPAS_INFO filtered BMS current/voltage and module status, never motor torque.

  Freshness comes from the actual received CAN frame. In particular, reading a
  retained parser value must not turn missing/stale data into a valid zero. The
  publisher masks this with already-computed CarState.canValid; do not read
  cp.can_valid here, because its getter advances the CAN-invalid debounce.
  """
  feedback.dataAvailable = False
  feedback.estimatedCurrentAmps = 0.0
  feedback.voltage = 0.0
  feedback.moduleStatus = 0
  feedback.sourceMonoTime = 0
  try:
    names = ("SteMdule_I_Est", "SteMdule_U_Meas", "SteMdule_D_Stat")
    times = [cp.ts_nanos.get("EPAS_INFO", {}).get(name, 0) for name in names]
    source_time = times[0]
    if not isinstance(source_time, int) or not 0 < source_time < 2**64 or any(t != source_time for t in times):
      return
    feedback.sourceMonoTime = source_time
    if not 0 <= now_nanos - source_time <= EPS_MAX_AGE_NS:
      return
    current, voltage, status = [float(cp.vl["EPAS_INFO"][name]) for name in names]
    # DBC raw all-ones current/voltage values are Invalid; status 7 is NotUsed.
    if (not all(math.isfinite(v) for v in (current, voltage, status)) or
        not -64.0 <= current <= 140.700001 or not 6.0 <= voltage <= 18.700001 or
        status != int(status) or not 0 <= status <= 6):
      return
    feedback.estimatedCurrentAmps = current
    feedback.voltage = voltage
    feedback.moduleStatus = int(status)
    feedback.dataAvailable = True
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    # Optional diagnostics must not interrupt car-state processing.
    return


PSCM_MAX_AGE_NS = 150_000_000


def fill_pscm_status(feedback, cp, now_nanos, can_fd):
  """Log raw CAN-FD PSCM feature/driver status; never infer spare steering authority.

  All four signals must come from the same recent received frame. Publisher
  validity masking uses the existing control result, without reading can_valid.
  """
  feedback.dataAvailable = False
  feedback.laActAvail = 0
  feedback.laActDeny = False
  feedback.laHandsOff = False
  feedback.tjaHandsOnConfidence = False
  feedback.sourceMonoTime = 0
  if not can_fd:
    return
  try:
    name = "Lane_Assist_Data3_FD1"
    signals = ("LaActAvail_D_Actl", "LaActDeny_B_Actl", "LaHandsOff_B_Actl", "TjaHandsOnCnfdnc_B_Est")
    times = [cp.ts_nanos.get(name, {}).get(signal, 0) for signal in signals]
    source_time = times[0]
    if not isinstance(source_time, int) or not 0 < source_time < 2**64 or any(t != source_time for t in times):
      return
    feedback.sourceMonoTime = source_time
    if not 0 <= now_nanos - source_time <= PSCM_MAX_AGE_NS:
      return
    values = [float(cp.vl[name][signal]) for signal in signals]
    if any(not math.isfinite(value) or value != int(value) or not 0 <= value <= maximum
           for value, maximum in zip(values, (3, 1, 1, 1), strict=True)):
      return
    feedback.laActAvail = int(values[0])
    feedback.laActDeny = bool(values[1])
    feedback.laHandsOff = bool(values[2])
    feedback.tjaHandsOnConfidence = bool(values[3])
    feedback.dataAvailable = True
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return
