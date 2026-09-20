"""Ford-reported limit states, separate from the arc's estimated demand length."""
from dataclasses import dataclass


MAX_FEEDBACK_AGE_NS = 150_000_000  # five nominal 33 Hz frames; reject cached CAN data


@dataclass(frozen=True)
class SteeringLimitDisplay:
  label: str = "DEMAND / CAPACITY UNKNOWN"
  rgb: tuple[int, int, int] = (230, 230, 230)


def ford_limit_display(feedback, now_ns: int, active: bool, data_valid: bool) -> SteeringLimitDisplay:
  if not active:
    return SteeringLimitDisplay(label="")
  if not data_valid or not feedback.dataAvailable:
    return SteeringLimitDisplay()
  age = now_ns - feedback.sourceMonoTime
  if feedback.sourceMonoTime <= 0 or not 0 <= age <= MAX_FEEDBACK_AGE_NS or feedback.controlStatus != 2:
    return SteeringLimitDisplay()
  # Zero only says the PSCM has not reported a limit. It is not a percentage,
  # nor evidence that this signal detects every limit in BP path-angle mode.
  return {
    1: SteeringLimitDisplay("NEAR LIMIT", (255, 200, 0)),
    2: SteeringLimitDisplay("LIMIT REACHED", (255, 65, 65)),
    3: SteeringLimitDisplay("LIMIT / DRIVER INPUT", (255, 65, 65)),
  }.get(feedback.status, SteeringLimitDisplay())
