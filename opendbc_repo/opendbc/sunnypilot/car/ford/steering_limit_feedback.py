"""Read Ford's documented limit feedback for display/logging, never actuation."""
from opendbc.car.ford.values import FordFlags


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
