import struct
from unittest.mock import MagicMock, call

import pytest
import usb1

from openpilot.selfdrive.modeld import chestnut_startup as startup


def usb_probe(monkeypatch, payloads, vendor=0x3801):
  handle = MagicMock()
  handle.controlRead.side_effect = payloads
  context = MagicMock()
  context.openByVendorIDAndProductID.side_effect = lambda vid, pid, **kwargs: handle if vid == vendor else None
  context.__enter__.return_value = context
  monkeypatch.setattr(startup.usb1, 'USBContext', lambda: context)
  return handle, context


@pytest.mark.parametrize('vendor', [0x3801, 0xADD1])
def test_ready_probe_uses_only_telemetry_reads_and_closes_handle(monkeypatch, vendor):
  handle, context = usb_probe(monkeypatch, [struct.pack('<Hh?', 13200, 1500, False), b'\x78'], vendor)
  assert startup.probe_chestnut_ready()
  assert handle.mock_calls == [
    call.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100),
    call.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100),
    call.close(),
  ]
  context.__exit__.assert_called_once()


@pytest.mark.parametrize('voltage,fault,pcie,expected', [
  (4999, False, 0x78, False),
  (5000, False, 0x78, True),
  (13200, True, 0x78, False),
  (13200, False, 0, False),
  (13200, False, 0x77, False),
])
def test_readiness_preserves_power_fault_and_link_checks(monkeypatch, voltage, fault, pcie, expected):
  handle, _ = usb_probe(monkeypatch, [struct.pack('<Hh?', voltage, 1500, fault), bytes([pcie])])
  assert startup.probe_chestnut_ready() is expected
  handle.close.assert_called_once()


@pytest.mark.parametrize('payloads', [
  [usb1.USBErrorTimeout()],
  [b''],
  [struct.pack('<Hh?', 13200, 1500, False), b''],
  [struct.pack('<Hh?', 13200, 1500, False), usb1.USBErrorNoDevice()],
])
def test_failed_or_short_read_cannot_start_model_and_releases_handle(monkeypatch, payloads):
  handle, context = usb_probe(monkeypatch, payloads)
  assert not startup.probe_chestnut_ready()
  handle.close.assert_called_once()
  context.__exit__.assert_called_once()


def test_absent_device_is_not_ready(monkeypatch):
  _, context = usb_probe(monkeypatch, [], vendor=0)
  assert not startup.probe_chestnut_ready()
  assert context.openByVendorIDAndProductID.call_count == len(startup.CHESTNUT_USB_IDS)


@pytest.mark.parametrize('ready_after', [0, 2, None])
def test_wait_retries_but_remains_bounded(monkeypatch, ready_after):
  elapsed = [0.]
  probes = []
  def probe():
    probes.append(elapsed[0])
    return ready_after is not None and len(probes) > ready_after
  monkeypatch.setattr(startup.time, 'monotonic', lambda: elapsed[0])
  monkeypatch.setattr(startup.time, 'sleep', lambda delay: elapsed.__setitem__(0, elapsed[0] + delay))
  monkeypatch.setattr(startup, 'probe_chestnut_ready', probe)
  assert startup.wait_for_chestnut_ready(0.25) is (ready_after is not None)
  assert 1 <= len(probes) <= 3
  assert elapsed[0] <= 0.25


@pytest.mark.parametrize('present,compiled,ready', [
  (True, True, True), (True, True, False), (False, True, True), (True, False, True),
])
def test_modeld_startup_does_not_wait_for_its_own_publisher(monkeypatch, present, compiled, ready):
  from openpilot.selfdrive.modeld import modeld

  class StartupCaptured(Exception):
    pass

  def stop_before_cameras(*args):
    raise StartupCaptured

  params = MagicMock()
  probe = MagicMock(return_value=ready)
  monkeypatch.setattr(modeld, 'Params', lambda: params)
  monkeypatch.setattr(modeld, 'chestnut_present', lambda: present)
  monkeypatch.setattr(modeld, 'chestnut_compiled', lambda: compiled)
  monkeypatch.setattr(modeld, 'wait_for_chestnut_ready', probe, raising=False)
  monkeypatch.setattr(modeld, 'config_realtime_process', stop_before_cameras)
  # A startup subscriber cannot receive modeld's later publication on a cold start.
  monkeypatch.setattr(modeld.messaging, 'sub_sock', MagicMock(side_effect=AssertionError('startup depends on its own publisher')))
  monkeypatch.setenv('HCQDEV_WAIT_TIMEOUT_MS', '10000')
  with pytest.raises(StartupCaptured):
    modeld.main()

  available = present and compiled
  params.put_bool.assert_any_call('ChestnutLoading', available and ready)
  if available:
    probe.assert_called_once_with(4. / modeld.SERVICE_LIST['deviceState'].frequency)
  else:
    probe.assert_not_called()
  if available and not ready:
    params.put_bool.assert_any_call('ChestnutActive', False)
    params.remove.assert_not_called()
  else:
    params.remove.assert_called_once_with('ChestnutActive')
