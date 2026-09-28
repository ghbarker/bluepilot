"""Status must distinguish live learning from saved evidence and stale telemetry."""
import json

import pytest

from openpilot.selfdrive.ui.bp.angle_autocal_status import calibration_status_text


def status(**extra):
  return json.dumps({"low": {"ph": "collect"}, "high": {"ph": "collect"}, **extra})


@pytest.mark.parametrize("raw", ["locked", status(pause="delay"), "garbage"])
def test_disabled_wins_over_old_status(raw):
  assert calibration_status_text(False, raw, onroad=True) == "Off"


def test_saved_lock_cannot_be_presented_as_live_when_publisher_is_missing():
  saved = json.dumps({"phase": "locked", "pipe": {}})
  assert calibration_status_text(True, None, saved) == "Saved: locked"
  assert calibration_status_text(True, None, saved, onroad=True) == "Status unavailable"
  assert calibration_status_text(True, "locked", saved, onroad=True) == "Locked"
  saved = json.dumps({"phase": "locked", "pipe": {"verify": [{"to": 1.05}, None]}})
  assert calibration_status_text(True, None, saved) == "Saved: adjustment under review"


@pytest.mark.parametrize("pause,expected", [("delay", "Waiting for steering delay"), ("inactive", "Waiting for active steering")])
def test_pause_takes_precedence_over_pending_trial(pause, expected):
  assert calibration_status_text(True, status(pause=pause, low={"ph": "verify"})) == expected


def test_trial_recovery_and_confirmed_response_are_distinct():
  assert calibration_status_text(True, status(low={"ph": "verify"})) == "Testing adjustment"
  assert calibration_status_text(True, status(low={"ph": "propose", "rollback": True})) == "Reverting adjustment"
  assert calibration_status_text(True, status(low={"ph": "collect", "vr": "failed"})) == "Collecting after rollback"
  assert calibration_status_text(True, status(low={"ph": "good"}, high={"ph": "good"})) == "Response matched; monitoring"


@pytest.mark.parametrize('reason,expected', [
  ('fit_evidence', 'High-speed: more turn data needed'),
  ('fit_confidence', 'High-speed: response inconsistent'),
  ('fresh_evidence', 'High-speed: collecting clean turns'),
  ('clean_frame', 'High-speed: waiting for clean data'),
  ('speed_range', 'High-speed: waiting for speed range'),
  ('settling', 'High-speed: waiting between trials'),
])
def test_actual_active_band_blocker_is_explained(reason, expected):
  raw = status(active='high', low={'ph': 'collect', 'reason': 'fit_evidence'},
               high={'ph': 'collect', 'reason': reason})
  assert calibration_status_text(True, raw) == expected


@pytest.mark.parametrize('reason,expected', [
  ('matching_turns', 'Testing: matching turns needed'),
  ('response_consistency', 'Testing: response inconsistent'),
])
def test_trial_blocker_is_explained(reason, expected):
  assert calibration_status_text(True, status(low={'ph': 'verify', 'reason': reason})) == expected


@pytest.mark.parametrize("raw", ["", "not json", "null", "[]", '{"low":null,"high":[]}', '{"low":{},"high":{}}', "{}"])
def test_unusable_live_payload_is_not_collection(raw):
  assert calibration_status_text(True, raw, onroad=True) == "Status unavailable"


@pytest.mark.parametrize("saved", ["", "garbage", "null", "[]", '{"phase":"locked","pipe":null}'])
def test_malformed_saved_state_does_not_crash(saved):
  assert calibration_status_text(True, None, saved) in ("Waiting for drive", "Saved status unavailable")
