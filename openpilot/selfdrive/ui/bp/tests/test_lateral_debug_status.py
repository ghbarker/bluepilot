import json
from types import SimpleNamespace
import unittest

from openpilot.selfdrive.ui.bp.lateral_debug_status import diagnostic_lines


class SM(dict):
  def __init__(self):
    super().__init__(lateralDelay=SimpleNamespace(lateralDelay=.372, status='estimated'),
                     controllerStateBP=SimpleNamespace(bmsAngleAutoCalibrate=True, bmsAngleAutoCalState='locked'))
    self.valid = {key: True for key in self}
    self.alive = {key: True for key in self}
    self.logMonoTime = {key: 1_000_000_000 for key in self}


class TestLateralDebugStatus(unittest.TestCase):
  def test_current_topic_live_delay_and_calibration(self):
    self.assertEqual(diagnostic_lines(SM(), 1_000_000_000, True, .1),
                     ('Delay 0.372s live', 'Auto-cal: Locked'))

  def test_stale_invalid_dead_and_future_messages_do_not_look_live(self):
    for flag in ('stale', 'valid', 'alive', 'future'):
      with self.subTest(flag=flag):
        sm = SM()
        if flag in ('valid', 'alive'):
          setattr(sm, flag, {key: False for key in sm})
        now = 3_000_000_001 if flag == 'stale' else 999_999_999 if flag == 'future' else 1_000_000_000
        self.assertEqual(diagnostic_lines(sm, now, True, .1),
                         ('Delay 0.100s default', 'Auto-cal: unavailable'))

  def test_offroad_does_not_reuse_previous_drive(self):
    self.assertEqual(diagnostic_lines(SM(), 1_000_000_000, False, .1),
                     ('Delay 0.100s default', 'Auto-cal: waiting for drive'))

  def test_nonfinite_and_malformed_delay(self):
    for value in (float('nan'), float('inf'), -1, 0, 'bad', None):
      with self.subTest(value=value):
        sm = SM()
        sm['lateralDelay'].lateralDelay = value
        self.assertEqual(diagnostic_lines(sm, 1_000_000_000, True)[0], 'Delay invalid')
        sm.alive['lateralDelay'] = False
        self.assertEqual(diagnostic_lines(sm, 1_000_000_000, True, value)[0], 'Delay unavailable')

  def test_learning_is_not_presented_as_estimated(self):
    sm = SM()
    sm['lateralDelay'].status = 'unestimated'
    self.assertEqual(diagnostic_lines(sm, 1_000_000_000, True)[0], 'Delay learning')

  def test_malformed_calibration_is_unavailable(self):
    for raw in (None, [], '{}', '{', '{"low": [], "high": 2}',
                '{"low":{"ph":"collect","reason":[]},"high":{"ph":"collect"}}'):
      with self.subTest(raw=raw):
        sm = SM()
        sm['controllerStateBP'].bmsAngleAutoCalState = raw
        self.assertIn('unavailable', diagnostic_lines(sm, 1_000_000_000, True)[1].lower())

  def test_evidence_can_decay_without_false_completion(self):
    sm = SM()
    for weight in (5., 4.):
      band = {'ph': 'collect', 'w': weight, 'need': 10}
      sm['controllerStateBP'].bmsAngleAutoCalState = json.dumps({'low': band, 'high': band})
      line = diagnostic_lines(sm, 1_000_000_000, True)[1]
      self.assertIn(f'Low {weight:.1f}/10.0', line)
      self.assertNotIn('Locked', line)

  def test_nonfinite_evidence_is_not_rendered(self):
    sm = SM()
    for weight in (float('nan'), float('inf'), -1):
      band = {'ph': 'collect', 'w': weight, 'need': 10}
      sm['controllerStateBP'].bmsAngleAutoCalState = json.dumps({'low': band, 'high': band})
      self.assertEqual(diagnostic_lines(sm, 1_000_000_000, True)[1], 'Auto-cal: Collecting response data')

  def test_pause_rollback_and_disabled_keep_existing_meaning(self):
    sm = SM()
    for raw, text in (('{"pause":"delay"}', 'Waiting for steering delay'),
                      ('{"low":{"rollback":true},"high":{}}', 'Reverting adjustment')):
      sm['controllerStateBP'].bmsAngleAutoCalState = raw
      self.assertEqual(diagnostic_lines(sm, 1_000_000_000, True)[1], f'Auto-cal: {text}')
    sm['controllerStateBP'].bmsAngleAutoCalibrate = False
    self.assertEqual(diagnostic_lines(sm, 1_000_000_000, True)[1], 'Auto-cal: Off')
