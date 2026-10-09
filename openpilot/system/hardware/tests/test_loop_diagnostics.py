from types import SimpleNamespace

import pytest

from openpilot.system.hardware.loop_diagnostics import HardwareLoopDiagnostics


def harness():
  now = [10.]
  monitor = HardwareLoopDiagnostics(clock=lambda: now[0])
  frame = SimpleNamespace(f_code=SimpleNamespace(co_filename='hardwared.py', co_name='hardware_thread'),
                          f_lineno=123, f_back=None)
  return now, monitor, {42: frame}


def test_startup_and_healthy_publications_do_not_generate_diagnostics():
  now, monitor, frames = harness()
  assert monitor.poll(42, frames) is None
  for _ in range(100):
    monitor.published()
    now[0] += .5
    assert monitor.poll(42, frames) is None


def test_stall_reports_once_and_real_recovery_gap_survives_late_poll():
  now, monitor, frames = harness()
  monitor.published()
  now[0] = 12.
  assert monitor.poll(42, frames) is None
  now[0] = 12.01
  result = monitor.poll(42, frames)
  assert result == {'event': 'hardwareStatusStalled', 'since_publication_s': pytest.approx(2.01),
                    'stack': [{'file': 'hardwared.py', 'line': 123, 'function': 'hardware_thread'}]}
  now[0] = 18.
  assert monitor.poll(42, frames) is None
  monitor.published()
  now[0] = 18.5
  monitor.published()
  now[0] = 18.9
  assert monitor.poll(42, frames) == {'event': 'hardwareStatusResumed', 'publication_gap_s': 8.}
  assert monitor.poll(42, frames) is None
  now[0] = 21.
  assert monitor.poll(42, frames)['event'] == 'hardwareStatusStalled'


def test_missing_thread_stack_can_be_retried():
  now, monitor, frames = harness()
  monitor.published()
  now[0] = 15.
  assert monitor.poll(None, frames) is None
  assert monitor.poll(42, {}) is None
  assert monitor.poll(42, frames)['event'] == 'hardwareStatusStalled'


def test_supervisor_missing_entire_stall_reports_gap_without_a_misleading_stack():
  now, monitor, frames = harness()
  monitor.published()
  now[0] = 18.
  monitor.published()
  now[0] = 18.5
  monitor.published()
  result = monitor.poll(42, frames)
  assert result == {'event': 'hardwareStatusGap', 'publication_gap_s': 8., 'stack_unavailable': True}
  assert monitor.poll(42, frames) is None
  now[0] = 24.
  monitor.published()
  assert monitor.poll(42, frames)['publication_gap_s'] == 5.5


def test_progress_during_capture_discards_the_stale_sample():
  now, monitor, frames = harness()
  monitor.published()
  now[0] = 15.

  class RacingFrames:
    def get(self, thread_id):
      monitor.published()
      return frames[thread_id]

  assert monitor.poll(42, RacingFrames()) is None
  assert monitor.reported_publication is None


def test_stack_is_bounded_and_contains_no_local_variables():
  now, monitor, _ = harness()

  class Frame:
    f_code = SimpleNamespace(co_filename='hardwared.py', co_name='hardware_thread')
    f_lineno = 5

    @property
    def f_back(self):
      return self

    @property
    def f_locals(self):
      raise AssertionError('Never inspect local variables')

  monitor.published()
  now[0] = 15.
  result = monitor.poll(42, {42: Frame()})
  assert len(result['stack']) == monitor.MAX_STACK_FRAMES
  assert all(set(row) == {'file', 'line', 'function'} for row in result['stack'])


def test_never_changes_publication_progress_itself():
  now, monitor, frames = harness()
  monitor.published()
  original = monitor.last_publication
  for t in (13., 15., 20., 60.):
    now[0] = t
    monitor.poll(42, frames)
    assert monitor.last_publication == original
