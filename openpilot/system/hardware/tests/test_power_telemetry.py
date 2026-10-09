import threading
from unittest.mock import Mock

import pytest

from openpilot.system.hardware.power_telemetry import SomPowerTelemetry


def test_sample_freshness_and_recovery():
  now = [10.]
  read = Mock(return_value=4.5)
  telemetry = SomPowerTelemetry(read, clock=lambda: now[0])
  assert telemetry.get() is None
  read.assert_not_called()
  telemetry.sample()
  assert telemetry.get() == 4.5
  now[0] = 12.
  assert telemetry.get() == 4.5
  now[0] = 12.001
  assert telemetry.get() is None
  telemetry.sample()
  assert telemetry.get() == 4.5
  now[0] = 1.
  assert telemetry.get() is None


def test_blocked_current_read_cannot_refresh_old_voltage_timestamp():
  now = [10.]

  def slow_read():
    now[0] += 8.
    return 4.5

  telemetry = SomPowerTelemetry(slow_read, clock=lambda: now[0])
  telemetry.sample()
  assert telemetry.get() is None


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), 0., OSError('sensor read failed')])
def test_invalid_or_failed_measurement_is_not_reused(bad):
  read = Mock(side_effect=[4.5, bad, 5.])
  telemetry = SomPowerTelemetry(read)
  telemetry.sample()
  assert telemetry.get() == 4.5
  telemetry.sample()
  assert telemetry.get() is None
  telemetry.sample()
  assert telemetry.get() == 5.


def test_signed_sensor_reading_is_preserved():
  telemetry = SomPowerTelemetry(lambda: -2.)
  telemetry.sample()
  assert telemetry.get() == -2.


def test_only_one_read_in_flight_and_consumers_do_not_wait():
  entered, release, stop = threading.Event(), threading.Event(), threading.Event()
  calls = []

  def blocked_read():
    calls.append(True)
    entered.set()
    assert release.wait(5.), 'test cleanup failed to release sensor'
    return 4.5

  telemetry = SomPowerTelemetry(blocked_read)
  thread = threading.Thread(target=telemetry.run, args=(stop,), daemon=True)
  thread.start()
  try:
    assert entered.wait(2.)
    for _ in range(100):
      assert telemetry.get() is None
    stop.set()
    assert thread.is_alive()  # The blocked reader is isolated, not cancelled.
    assert len(calls) == 1
  finally:
    stop.set()
    release.set()
    thread.join(2.)
  assert not thread.is_alive()
  assert len(calls) == 1
