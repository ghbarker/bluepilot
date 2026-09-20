import os
from unittest.mock import Mock

import pytest

from openpilot.common.serial import Serial, SerialException
from openpilot.system.qcomgpsd import modemdiag, qcomgpsd as gps


@pytest.mark.parametrize('initial', ['+QGPSCFG: "gnssconfig",4', ''])
def test_gnss_configuration_is_verified(initial, monkeypatch):
  command = Mock(side_effect=[initial, '', '+QGPSCFG: "gnssconfig",1'])
  monkeypatch.setattr(gps, 'at_cmd', command)
  gps.configure_gnss()
  assert [c.args[0] for c in command.call_args_list] == [
    'AT+QGPSCFG="gnssconfig"', 'AT+QGPSCFG="gnssconfig",1', 'AT+QGPSCFG="gnssconfig"']


def test_already_configured_gnss_does_not_rewrite_nv(monkeypatch):
  command = Mock(return_value='+QGPSCFG: "gnssconfig",1')
  monkeypatch.setattr(gps, 'at_cmd', command)
  gps.configure_gnss()
  command.assert_called_once()


def test_unsupported_constellation_setting_keeps_gps_startup_available(monkeypatch):
  monkeypatch.setattr(gps, 'at_cmd', Mock(return_value=''))
  logger = Mock()
  monkeypatch.setattr(gps, 'cloudlog', logger)
  gps.configure_gnss()
  logger.warning.assert_called_once()


@pytest.mark.parametrize('error', [SerialException('unplugged'), OSError('select failed')])
def test_recv_recovers_and_does_not_return_stale_payload(monkeypatch, error):
  old = Mock()
  old.recv.side_effect = error
  new = Mock()
  new.recv.return_value = (16, b'fresh data')
  monkeypatch.setattr(gps, '_read_modem_state', lambda: {})
  reconnect = Mock(return_value=new)
  monkeypatch.setattr(gps, '_reconnect_diag', reconnect)
  assert gps.recv_diag(old) == (new, 16, b'fresh data')
  old.serial.close.assert_called_once()
  reconnect.assert_called_once()


def test_reconnect_closes_failed_setup_before_retry(monkeypatch):
  first, second = Mock(), Mock()
  monkeypatch.setattr(gps, 'ModemDiag', Mock(side_effect=[first, second]))
  monkeypatch.setattr(gps, 'wait_for_modem', Mock())
  setup = Mock(side_effect=[RuntimeError('modem resetting'), None])
  monkeypatch.setattr(gps, 'setup_quectel', setup)
  monkeypatch.setattr(gps.time, 'sleep', Mock())
  assert gps._reconnect_diag() is second
  first.serial.close.assert_called_once()
  second.serial.close.assert_not_called()
  assert setup.call_count == 2


def test_diagnostic_port_eof_raises_instead_of_spinning():
  # A real Linux PTY models the new Serial implementation's empty read on HUP.
  master, slave = os.openpty()
  port = Serial(os.ttyname(slave), timeout=0)
  os.close(slave)
  diag = modemdiag.ModemDiag.__new__(modemdiag.ModemDiag)
  diag.serial, diag.pend = port, b''
  os.close(master)
  try:
    with pytest.raises(SerialException):
      diag.recv()
  finally:
    port.close()


def test_failed_port_initialization_releases_exclusive_handle(monkeypatch):
  port = Mock()
  port.flush.side_effect = OSError('modem disappeared')
  monkeypatch.setattr(modemdiag, 'Serial', Mock(return_value=port))
  with pytest.raises(OSError):
    modemdiag.ModemDiag()
  port.close.assert_called_once()


@pytest.mark.parametrize('contents', ['not json', '[]', '{}'])
def test_modem_diagnostics_tolerate_invalid_state(tmp_path, monkeypatch, contents):
  path = tmp_path / 'modem'
  path.write_text(contents)
  monkeypatch.setattr(gps, 'MODEM_STATE_PATH', str(path))
  assert gps._read_modem_state() == {}
