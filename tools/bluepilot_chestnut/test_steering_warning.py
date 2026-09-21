"""Recorded Ford warning cases: preserve requests, distinguish live trouble from a retained alert."""
import copy
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from openpilot.cereal import log, messaging
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import EVENTS, EventName, ET
from openpilot.selfdrive.selfdrived.ford_steering_alert import FordSteeringAlert, MAX_DATA_AGE_NS
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EmptyAlert, ImmediateDisableAlert


NOW = 10_000_000_000


class Messages(dict):
  def refresh(self, now):
    self.valid = dict.fromkeys(self, True)
    self.logMonoTime = dict.fromkeys(self, now)
    self['carStateBP'].fordSteeringLimit.sourceMonoTime = now


def inputs():
  car = structs.CarState.new_message(vEgo=22.97, canValid=True)
  controls = log.ControlsState.new_message(curvature=-0.00121)
  angle = controls.init('lateralControlState').init('angleState')
  angle.active = True
  angle.steeringAngleDesiredDeg = 4.04
  angle.steeringAngleDeg = 0.
  angle.saturated = True
  cc = structs.CarControl.new_message(latActive=True)
  cc.actuators.curvature = -0.00228
  model = log.ModelDataV2.new_message()
  model.action.desiredCurvature = -0.00228
  ford = messaging.new_message('carStateBP').carStateBP
  ford.fordSteeringLimit.dataAvailable = True
  ford.fordSteeringLimit.controlStatus = 2
  sm = Messages(controlsState=controls, carControl=cc, modelV2=model, carStateBP=ford)
  sm.refresh(NOW)
  return car, sm


def warning():
  alert = copy.copy(EVENTS[EventName.steerSaturated][ET.WARNING])
  alert.alert_type = 'steerSaturated/warning'
  alert.event_type = ET.WARNING
  return alert


def recover(sm):
  angle = sm['controlsState'].lateralControlState.angleState
  angle.saturated = False
  angle.steeringAngleDesiredDeg = -0.12
  sm['controlsState'].curvature = -0.00204
  sm['modelV2'].action.desiredCurvature = -0.00201


def step(presenter, alert, car, sm, frame, event_active=False, refresh=True, car_timestamp=None):
  now = NOW + frame * 10_000_000
  if refresh:
    sm.refresh(now)
  return presenter.update(alert, event_active, frame, now, car, now if car_timestamp is None else car_timestamp, sm)


@pytest.mark.parametrize('mici', [False, True])
def test_tracking_shortfall_stays_loud_even_when_ford_reports_no_limit(mici):
  car, sm = inputs()
  presenter, alert = FordSteeringAlert(mici), warning()
  for frame in range(300):
    shown = step(presenter, alert, car, sm, frame, event_active=True)
    assert shown.alert_text_1.lower() == 'take control'
    assert shown.alert_text_2.lower() == 'steering not keeping up'
    for key in ('audible_alert', 'priority', 'alert_status', 'alert_size', 'visual_alert', 'alert_type', 'event_type', 'duration'):
      assert getattr(shown, key) == getattr(alert, key)


def test_recorded_transient_recovers_without_extending_or_shortening_visual_lifetime():
  car, sm = inputs()
  presenter, alert = FordSteeringAlert(), warning()
  manager = AlertManager()
  original = copy.copy(alert).__dict__
  requests = sm['carControl'].as_reader().as_builder().to_bytes()
  for frame in range(205):
    active = frame < 16  # Recorded trigger cleared ~0.16 s after onset.
    if active:
      manager.add_many(frame, [alert])
    else:
      recover(sm)
      if frame >= 150:
        # A later threshold crossing must not revive the retained warning sound
        # or change when AlertManager removes the original visual alert.
        sm['controlsState'].lateralControlState.angleState.steeringAngleDesiredDeg = -2.57
    manager.process_alerts(frame, set())
    selected = manager.current_alert
    shown = step(presenter, selected, car, sm, frame, event_active=active)
    assert (shown is EmptyAlert) == (selected is EmptyAlert)
    assert sm['carControl'].as_reader().as_builder().to_bytes() == requests
    if frame == 45:
      assert shown.alert_text_1 == 'Steering Alert'
      assert shown.alert_text_2 == 'Check Steering Response'
      assert shown.audible_alert == alert.audible_alert  # Initial tone still delivered.
    if 100 <= frame <= 200:
      assert shown.audible_alert == log.SelfdriveState.AudibleAlert.none
      assert shown.visual_alert == alert.visual_alert
  assert alert.__dict__ == original  # Never mutate the shared alert definition/entry.


@pytest.mark.parametrize('status,text', [(1, 'Near Steering Limit'), (2, 'Steering Limit Reached'), (3, 'Steering Limit Reached')])
def test_reported_limit_keeps_takeover_noise_even_after_event_clears(status, text):
  car, sm = inputs()
  recover(sm)
  sm['carStateBP'].fordSteeringLimit.status = status
  presenter, alert = FordSteeringAlert(), warning()
  for frame in range(210):
    shown = step(presenter, alert, car, sm, frame)
    assert shown.alert_text_2 == text
    assert shown.audible_alert == alert.audible_alert


@pytest.mark.parametrize('condition', [
  'driver', 'fault', 'permanent', 'invalid_can', 'car_stale', 'controls_stale', 'model_stale', 'control_stale',
  'feedback_stale', 'feedback_invalid', 'feedback_missing', 'feedback_future', 'ramp_out', 'error', 'nan_error',
  'nan_speed', 'low_speed', 'opposite_direction', 'large_opposite_response', 'inactive', 'undershooting',
])
def test_event_clearing_alone_does_not_prove_recovery(condition):
  car, sm = inputs()
  recover(sm)
  presenter, alert = FordSteeringAlert(), warning()
  for frame in range(160):
    now = NOW + frame * 10_000_000
    sm.refresh(now)
    timestamp = now
    if condition == 'driver':
      car.steeringPressed = True
    elif condition == 'fault':
      car.steerFaultTemporary = True
    elif condition == 'permanent':
      car.steerFaultPermanent = True
    elif condition == 'invalid_can':
      car.canValid = False
    elif condition == 'car_stale':
      timestamp = now - MAX_DATA_AGE_NS - 1
    elif condition in ('controls_stale', 'model_stale', 'control_stale'):
      service = {'controls_stale': 'controlsState', 'model_stale': 'modelV2', 'control_stale': 'carControl'}[condition]
      sm.logMonoTime[service] = now - MAX_DATA_AGE_NS - 1
    elif condition == 'feedback_stale':
      sm['carStateBP'].fordSteeringLimit.sourceMonoTime = now - MAX_DATA_AGE_NS - 1
    elif condition == 'feedback_invalid':
      sm.valid['carStateBP'] = False
    elif condition == 'feedback_missing':
      sm['carStateBP'].fordSteeringLimit.dataAvailable = False
    elif condition == 'feedback_future':
      sm['carStateBP'].fordSteeringLimit.sourceMonoTime = now + 1
    elif condition == 'ramp_out':
      sm['carStateBP'].fordSteeringLimit.controlStatus = 3
    elif condition in ('error', 'nan_error'):
      sm['controlsState'].lateralControlState.angleState.steeringAngleDesiredDeg = 10. if condition == 'error' else float('nan')
    elif condition == 'nan_speed':
      car.vEgo = float('nan')
    elif condition == 'low_speed':
      car.vEgo = 4.9
    elif condition == 'opposite_direction':
      sm['controlsState'].curvature = 0.00204
    elif condition == 'large_opposite_response':
      sm['controlsState'].curvature = 0.005
      sm['modelV2'].action.desiredCurvature = -0.0002
    elif condition == 'inactive':
      sm['carControl'].latActive = False
    elif condition == 'undershooting':
      sm['controlsState'].curvature = -0.001
    shown = step(presenter, alert, car, sm, frame, refresh=False, car_timestamp=timestamp)
    assert shown.alert_text_1 == 'Take Control'
    assert shown.audible_alert == alert.audible_alert


def test_recurrent_bad_tracking_restores_noise_immediately():
  car, sm = inputs()
  recover(sm)
  presenter, alert = FordSteeringAlert(), warning()
  for frame in range(110):
    shown = step(presenter, alert, car, sm, frame)
  assert shown.audible_alert == log.SelfdriveState.AudibleAlert.none
  shown = step(presenter, alert, car, sm, 110, event_active=True)
  assert shown.alert_text_1 == 'Take Control'
  assert shown.audible_alert == alert.audible_alert


@pytest.mark.parametrize('mici', [False, True])
@pytest.mark.parametrize('fluctuation', ['angle_error', 'saturation_timer', 'undershooting'])
def test_recovered_warning_does_not_restart_sound_without_a_new_warning(mici, fluctuation):
  # 2026-09-21 07:30:56: the recorded event had cleared and sound was off,
  # then a tracking fluctuation restarted promptRepeat inside the retained alert.
  car, sm = inputs()
  recover(sm)
  presenter, alert = FordSteeringAlert(mici), warning()
  original = copy.copy(alert).__dict__
  requests = sm['carControl'].as_reader().as_builder().to_bytes()
  for frame in range(140):
    shown = step(presenter, alert, car, sm, frame)
  assert shown.audible_alert == log.SelfdriveState.AudibleAlert.none
  angle = sm['controlsState'].lateralControlState.angleState
  if fluctuation == 'angle_error':
    angle.steeringAngleDesiredDeg = -2.57
  elif fluctuation == 'saturation_timer':
    angle.saturated = True
  else:
    sm['controlsState'].curvature = -0.001
  for frame in range(140, 190):
    shown = step(presenter, alert, car, sm, frame)
    assert shown.audible_alert == log.SelfdriveState.AudibleAlert.none
    assert shown.alert_text_2.lower() == 'check steering response'
    assert shown.visual_alert == alert.visual_alert
    assert shown.duration == alert.duration
    assert sm['carControl'].as_reader().as_builder().to_bytes() == requests
  assert alert.__dict__ == original


@pytest.mark.parametrize('condition', [
  'event', 'near_limit', 'limit', 'driver_limit', 'fault', 'permanent', 'driver', 'invalid_can',
  'car_stale', 'model_stale', 'controls_stale', 'control_stale', 'feedback_stale', 'feedback_invalid',
  'feedback_missing', 'feedback_future', 'ramp_out', 'inactive', 'low_speed', 'nan', 'opposite_response',
])
def test_recovered_warning_rearms_for_new_warning_or_unverifiable_control(condition):
  car, sm = inputs()
  recover(sm)
  presenter, alert = FordSteeringAlert(), warning()
  for frame in range(140):
    shown = step(presenter, alert, car, sm, frame)
  assert shown.audible_alert == log.SelfdriveState.AudibleAlert.none
  now = NOW + 140 * 10_000_000
  sm.refresh(now)
  timestamp = now
  feedback = sm['carStateBP'].fordSteeringLimit
  if condition in ('near_limit', 'limit', 'driver_limit'):
    feedback.status = {'near_limit': 1, 'limit': 2, 'driver_limit': 3}[condition]
  elif condition in ('fault', 'permanent'):
    setattr(car, 'steerFaultTemporary' if condition == 'fault' else 'steerFaultPermanent', True)
  elif condition == 'driver':
    car.steeringPressed = True
  elif condition == 'invalid_can':
    car.canValid = False
  elif condition == 'car_stale':
    timestamp = now - MAX_DATA_AGE_NS - 1
  elif condition in ('model_stale', 'controls_stale', 'control_stale'):
    service = {'model_stale': 'modelV2', 'controls_stale': 'controlsState', 'control_stale': 'carControl'}[condition]
    sm.logMonoTime[service] = now - MAX_DATA_AGE_NS - 1
  elif condition == 'feedback_stale':
    feedback.sourceMonoTime = now - MAX_DATA_AGE_NS - 1
  elif condition == 'feedback_invalid':
    sm.valid['carStateBP'] = False
  elif condition == 'feedback_missing':
    feedback.dataAvailable = False
  elif condition == 'feedback_future':
    feedback.sourceMonoTime = now + 1
  elif condition == 'ramp_out':
    feedback.controlStatus = 3
  elif condition == 'inactive':
    sm['carControl'].latActive = False
  elif condition == 'low_speed':
    car.vEgo = 4.9
  elif condition == 'nan':
    sm['controlsState'].lateralControlState.angleState.steeringAngleDesiredDeg = float('nan')
  elif condition == 'opposite_response':
    sm['controlsState'].curvature = 0.005
  shown = step(presenter, alert, car, sm, 140, event_active=condition == 'event', refresh=False, car_timestamp=timestamp)
  assert shown.audible_alert == alert.audible_alert
  assert shown.alert_text_1 == 'Take Control'


def test_rearmed_warning_preserves_initial_sound_delivery_time():
  car, sm = inputs()
  recover(sm)
  presenter, alert = FordSteeringAlert(), warning()
  for frame in range(140):
    step(presenter, alert, car, sm, frame)
  assert step(presenter, alert, car, sm, 140, event_active=True).audible_alert == alert.audible_alert
  for frame in range(141, 240):
    assert step(presenter, alert, car, sm, frame).audible_alert == alert.audible_alert
  assert step(presenter, alert, car, sm, 240).audible_alert == log.SelfdriveState.AudibleAlert.none


@pytest.mark.parametrize('interruption', ['empty', 'critical', 'frame_gap'])
def test_recovered_sound_latch_does_not_leak_into_another_alert(interruption):
  car, sm = inputs()
  recover(sm)
  presenter, alert = FordSteeringAlert(), warning()
  for frame in range(140):
    step(presenter, alert, car, sm, frame)
  if interruption == 'empty':
    assert step(presenter, EmptyAlert, car, sm, 140) is EmptyAlert
  elif interruption == 'critical':
    critical = ImmediateDisableAlert('Steering Assist Temporarily Unavailable')
    critical.alert_type = 'steerTempUnavailable/immediateDisable'
    assert step(presenter, critical, car, sm, 140) is critical
  assert step(presenter, alert, car, sm, 141).audible_alert == alert.audible_alert


def test_good_requests_do_not_create_alerts_and_critical_alerts_are_untouched():
  car, sm = inputs()
  presenter = FordSteeringAlert()
  for frame in range(20):
    assert step(presenter, EmptyAlert, car, sm, frame) is EmptyAlert
  critical = ImmediateDisableAlert('Steering Assist Temporarily Unavailable')
  critical.alert_type = 'steerTempUnavailable/immediateDisable'
  assert step(presenter, critical, car, sm, 20) is critical


def test_missing_optional_service_is_not_recovery():
  car, sm = inputs()
  del sm['carStateBP']
  sm.valid.pop('carStateBP')
  sm.logMonoTime.pop('carStateBP')
  shown = FordSteeringAlert().update(warning(), False, 0, NOW, car, NOW, sm)
  assert shown.alert_text_1 == 'Take Control'


@pytest.mark.parametrize('brand', ['ford', 'toyota'])
def test_selfdrived_changes_only_selected_ford_presentation(brand):
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.selfdrive.selfdrived.events import Events

  car, sm = inputs()
  sm.frame = 0
  daemon = SelfdriveD.__new__(SelfdriveD)
  daemon.CP = SimpleNamespace(brand=brand)
  daemon.sm = sm
  daemon.enabled = True
  daemon.personality = log.LongitudinalPersonality.standard
  daemon.is_metric = True
  daemon.state_machine = SimpleNamespace(current_alert_types=[ET.WARNING], soft_disable_timer=0)
  daemon.events = Events()
  daemon.events.add(EventName.steerSaturated)
  daemon.events_sp = SimpleNamespace(create_alerts=lambda *args: [])
  daemon.AM = AlertManager()
  daemon.ford_steering_alert = FordSteeringAlert()
  daemon.car_state_timestamp = NOW
  before = list(daemon.events.names)
  with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=NOW):
    daemon.update_alerts(car)
  assert daemon.events.names == before
  assert daemon.enabled
  assert daemon.AM.current_alert.alert_type == 'steerSaturated/warning'
  if brand == 'ford':
    assert daemon.AM.current_alert.alert_text_2 == 'Steering Not Keeping Up'
  else:
    assert daemon.AM.current_alert.alert_text_2 == EVENTS[EventName.steerSaturated][ET.WARNING].alert_text_2
