"""Ford limit feedback for display and calibration admission, never actuator limits."""
from opendbc.car.ford.values import FordFlags


CALIBRATION_FEEDBACK_MAX_AGE_NS = 150_000_000


def calibration_feedback_clear(CS, flags, now_nanos):
  """CAN FD calibration needs a fresh, active, unrestricted steering response.

  This does not populate lat_ctl_lim_stat or activate the controller's dormant
  limit-clamp branches. Older CAN platforms retain their existing admission.
  """
  if not flags & FordFlags.CANFD:
    return True
  message = getattr(CS, 'car_state_bp_msg', None)
  if message is None or not message.valid or not CS.out.canValid:
    return False
  feedback = message.carStateBP.fordSteeringLimit
  return (feedback.dataAvailable and feedback.sourceMonoTime > 0
          and 0 <= now_nanos - feedback.sourceMonoTime <= CALIBRATION_FEEDBACK_MAX_AGE_NS
          and feedback.controlStatus == 2 and feedback.status == 0)


def fill_steering_limit_feedback(feedback, cp, flags):
  # Do not populate CS.lat_ctl_lim_stat: doing so would activate dormant BP
  # controller branches. This is deliberately a separate, read-only data path.
  if not flags & FordFlags.CANFD:
    return
  name = "Lane_Assist_Data3_FD1"
  timestamp = cp.ts_nanos.get(name, {}).get("LatCtlLim_D_Stat", 0)
  if timestamp <= 0:
    return
  values = cp.vl[name]
  feedback.status = int(values["LatCtlLim_D_Stat"])
  feedback.controlStatus = int(values["LatCtlSte_D_Stat"])
  feedback.sourceMonoTime = timestamp
  feedback.dataAvailable = True
