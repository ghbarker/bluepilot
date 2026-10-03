"""Optional telemetry must distinguish absent/stale input from valid zero data."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.sunnypilot.car.ford import steering_diagnostics
from opendbc.sunnypilot.car.ford.steering_diagnostics import can_clock_nanos, fill_eps_diagnostics, steering_command_snapshot


NOW = 10_000_000_000


def eps_parser():
  values = {"SteMdule_I_Est": 0.0, "SteMdule_U_Meas": 12.5, "SteMdule_D_Stat": 2}
  return SimpleNamespace(can_valid=True, vl={"EPAS_INFO": values}, ts_nanos={"EPAS_INFO": dict.fromkeys(values, NOW)})


class TestSteeringDiagnostics(unittest.TestCase):
  def test_valid_zero_current_and_received_timestamp(self):
    out = SimpleNamespace()
    fill_eps_diagnostics(out, eps_parser(), NOW + 100_000_000)
    self.assertTrue(out.dataAvailable)
    self.assertEqual((out.estimatedCurrentAmps, out.voltage, out.moduleStatus, out.sourceMonoTime), (0., 12.5, 2, NOW))

  def test_missing_stale_invalid_or_future_can_clears_previous_valid_data(self):
    for variant in ("missing", "stale", "future", "zero_timestamp", "mismatched_timestamps", "missing_signal"):
      with self.subTest(variant=variant):
        cp = eps_parser()
        out = SimpleNamespace()
        fill_eps_diagnostics(out, cp, NOW)
        self.assertTrue(out.dataAvailable)
        now = NOW
        if variant == "missing":
          cp = None
        elif variant == "stale":
          now += 100_000_001
        elif variant == "future":
          now -= 1
        elif variant == "zero_timestamp":
          cp.ts_nanos["EPAS_INFO"] = dict.fromkeys(cp.vl["EPAS_INFO"], 0)
        elif variant == "mismatched_timestamps":
          cp.ts_nanos["EPAS_INFO"]["SteMdule_U_Meas"] -= 1
        else:
          del cp.vl["EPAS_INFO"]["SteMdule_I_Est"]
        fill_eps_diagnostics(out, cp, now)
        self.assertFalse(out.dataAvailable)
        self.assertEqual((out.estimatedCurrentAmps, out.voltage, out.moduleStatus), (0., 0., 0))

  def test_dbc_sentinels_out_of_range_and_nonfinite_values(self):
    for signal, invalid in (("SteMdule_I_Est", 140.75), ("SteMdule_U_Meas", 18.75), ("SteMdule_D_Stat", 7),
                            ("SteMdule_I_Est", -64.05), ("SteMdule_U_Meas", 5.95), ("SteMdule_D_Stat", 2.5)):
      for value in (invalid, float("nan"), float("inf"), None, "bad"):
        with self.subTest(signal=signal, value=value):
          cp, out = eps_parser(), SimpleNamespace()
          cp.vl["EPAS_INFO"][signal] = value
          fill_eps_diagnostics(out, cp, NOW)
          self.assertFalse(out.dataAvailable)

  def test_module_failure_status_is_valid_diagnostic_information(self):
    cp, out = eps_parser(), SimpleNamespace()
    cp.vl["EPAS_INFO"].update(SteMdule_I_Est=-64., SteMdule_U_Meas=18.7, SteMdule_D_Stat=6)
    fill_eps_diagnostics(out, cp, NOW)
    self.assertTrue(out.dataAvailable)
    self.assertEqual(out.moduleStatus, 6)

  def test_optional_diagnostics_never_read_stateful_parser_validity(self):
    class Parser:
      vl = eps_parser().vl
      ts_nanos = eps_parser().ts_nanos

      @property
      def can_valid(self):
        raise AssertionError("optional telemetry must not advance parser debounce")

    out = SimpleNamespace()
    fill_eps_diagnostics(out, Parser(), NOW)
    self.assertTrue(out.dataAvailable)

  def test_can_clock_includes_suspend_time(self):
    # BOOTTIME source timestamps remain fresh after suspend even though Linux's
    # CLOCK_MONOTONIC excludes that interval.
    with patch.object(steering_diagnostics.time, "CLOCK_BOOTTIME", 7, create=True), \
         patch.object(steering_diagnostics.time, "clock_gettime_ns", return_value=NOW + 50_000_000, create=True) as boot, \
         patch.object(steering_diagnostics.time, "monotonic_ns", return_value=NOW - 3_000_000_000) as monotonic:
      out = SimpleNamespace()
      fill_eps_diagnostics(out, eps_parser(), can_clock_nanos())
      self.assertTrue(out.dataAvailable)
      self.assertEqual(out.sourceMonoTime, NOW)
      boot.assert_called_once_with(7)
      monotonic.assert_not_called()

  def test_can_clock_has_explicit_non_linux_fallback(self):
    fake_time = SimpleNamespace(monotonic_ns=lambda: NOW)
    with patch.object(steering_diagnostics, "time", fake_time):
      self.assertEqual(can_clock_nanos(), NOW)

  def test_bad_packet_cannot_raise_or_claim_available(self):
    for message in (None, (), (0x3D6, b"short", 0), (0x123, bytes(8), 0), (0x3D6, "12345678", 0)):
      with self.subTest(message=message):
        self.assertEqual(steering_command_snapshot(message, 1, NOW), {"dataAvailable": False})
    for frame, timestamp in ((-1, NOW), (0, 0), (2**64, NOW), (1, float("nan")), (1, 2**64)):
      self.assertEqual(steering_command_snapshot((0x3D6, bytes(8), 0), frame, timestamp), {"dataAvailable": False})


if __name__ == "__main__":
  unittest.main()
