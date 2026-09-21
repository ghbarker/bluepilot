"""Ford steering-warning presentation; never creates events or changes actuation."""
import copy
import math

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_angle import STEER_ANGLE_SATURATION_THRESHOLD


MAX_DATA_AGE_NS = 150_000_000
RECOVERY_FRAMES = round(0.3 / DT_CTRL)
MIN_SOUND_FRAMES = round(1.0 / DT_CTRL)  # Stock warning tone is 0.75 s; allow soundd delivery time.


def fresh(timestamp, now_ns):
  return timestamp > 0 and 0 <= now_ns - timestamp <= MAX_DATA_AGE_NS


class FordSteeringAlert:
  def __init__(self, mici=False):
    self.mici = mici
    self.first_frame = None
    self.last_frame = None
    self.recovery_frames = 0
    self.sound_silenced = False

  def update(self, alert, event_active, frame, now_ns, CS, car_state_timestamp, sm):
    if alert.alert_type != "steerSaturated/warning":
      self.first_frame = self.last_frame = None
      self.recovery_frames = 0
      self.sound_silenced = False
      return alert

    if self.last_frame is None or frame != self.last_frame + 1:
      self.first_frame = frame
      self.recovery_frames = 0
      self.sound_silenced = False
    self.last_frame = frame

    def valid(service):
      return sm.valid.get(service, False) and fresh(sm.logMonoTime.get(service, 0), now_ns)

    # Limit feedback is optional. Missing/old data never means spare capacity.
    limit = None
    if valid('carStateBP'):
      feedback = sm['carStateBP'].fordSteeringLimit
      if feedback.dataAvailable and feedback.controlStatus == 2 and fresh(feedback.sourceMonoTime, now_ns):
        limit = feedback.status

    displayed = copy.copy(alert)
    displayed.alert_text_1 = "Take Control"
    displayed.alert_text_2 = {
      1: "Near Steering Limit",
      2: "Steering Limit Reached",
      3: "Steering Limit Reached",
    }.get(limit, "Steering Not Keeping Up")

    tracking_recovered = False
    can_keep_quiet = False
    if (not event_active and limit == 0 and fresh(car_state_timestamp, now_ns) and CS.canValid
        and not CS.steeringPressed and not CS.steerFaultTemporary and not CS.steerFaultPermanent
        and all(valid(s) for s in ('controlsState', 'carControl', 'modelV2'))):
      controls = sm['controlsState']
      if controls.lateralControlState.which() == 'angleState':
        lateral = controls.lateralControlState.angleState
        error = lateral.steeringAngleDesiredDeg - lateral.steeringAngleDeg
        speed = max(CS.vEgo, 0.3)
        actual = controls.curvature * speed**2
        desired = sm['modelV2'].action.desiredCurvature * speed**2
        finite = all(math.isfinite(v) for v in (error, speed, actual, desired))
        # Clearing due to driver override, a stale message, low-speed gating,
        # or saturation-timer decay is not proof that steering caught up.
        gentle = abs(desired) <= 1.0 and abs(actual) <= 1.0
        following = (gentle
                     or (actual * desired > 0 and abs(desired) <= 1.2 * abs(1e-3 + actual)))
        can_keep_quiet = (finite and CS.vEgo > 5.0 and lateral.active and sm['carControl'].latActive
                          and (gentle or actual * desired > 0))
        tracking_recovered = (can_keep_quiet and not lateral.saturated
                              and abs(error) <= STEER_ANGLE_SATURATION_THRESHOLD and following)

    self.recovery_frames = self.recovery_frames + 1 if tracking_recovered else 0
    if not can_keep_quiet:
      # A new event, Ford limit, fault, override, or unverifiable control restores
      # the existing alert immediately. A restarted tone gets its delivery window.
      if self.sound_silenced:
        self.first_frame = frame
      self.sound_silenced = False
    if self.recovery_frames >= RECOVERY_FRAMES:
      if frame - self.first_frame >= MIN_SOUND_FRAMES:
        self.sound_silenced = True

    if self.recovery_frames >= RECOVERY_FRAMES or self.sound_silenced:
      # Retain the original visual lifetime, but describe a past warning rather
      # than an ongoing limit. This deliberately does not say "safe" or "all clear".
      displayed.alert_text_1 = "Steering Alert"
      displayed.alert_text_2 = "Check Steering Response"
      if self.sound_silenced:
        # Once recovery has quieted this retained alert, small tracking changes
        # must pass the normal warning detector before restarting its sound.
        # Never use this latch to qualify initial recovery or change an event.
        displayed.audible_alert = log.SelfdriveState.AudibleAlert.none

    if self.mici:
      displayed.alert_text_1 = displayed.alert_text_1.lower()
      displayed.alert_text_2 = displayed.alert_text_2.lower()
    return displayed
