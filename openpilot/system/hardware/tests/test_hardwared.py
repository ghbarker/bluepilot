import queue
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.cereal import log
from openpilot.system.hardware import hardwared
from openpilot.system.hardware.power_telemetry import SomPowerTelemetry


@pytest.fixture(autouse=True)
def clear_alert_cache():
  hardwared.prev_offroad_states.clear()
  yield
  hardwared.prev_offroad_states.clear()


def test_hidden_alert_details_do_not_repeat_storage_work(monkeypatch):
  persist = Mock()
  monkeypatch.setattr(hardwared, 'set_offroad_alert', persist)
  for temperature in ('40.0C', '41.0C', '42.0C'):
    hardwared.set_offroad_alert_if_changed('Offroad_TemperatureTooHigh', False, temperature)
  persist.assert_called_once_with('Offroad_TemperatureTooHigh', False, None)

  hardwared.set_offroad_alert_if_changed('Offroad_TemperatureTooHigh', True, '107.0C')
  hardwared.set_offroad_alert_if_changed('Offroad_TemperatureTooHigh', True, '108.0C')
  hardwared.set_offroad_alert_if_changed('Offroad_TemperatureTooHigh', False, '90.0C')
  assert persist.call_count == 4
  assert persist.call_args_list[1].args == ('Offroad_TemperatureTooHigh', True, '107.0C')
  assert persist.call_args_list[2].args == ('Offroad_TemperatureTooHigh', True, '108.0C')
  assert persist.call_args_list[3].args == ('Offroad_TemperatureTooHigh', False, None)


def test_failed_alert_write_can_retry(monkeypatch):
  persist = Mock(side_effect=[OSError('storage unavailable'), None])
  monkeypatch.setattr(hardwared, 'set_offroad_alert', persist)
  with pytest.raises(OSError):
    hardwared.set_offroad_alert_if_changed('Offroad_TiciSupport', True, 'staging')
  hardwared.set_offroad_alert_if_changed('Offroad_TiciSupport', True, 'staging')
  assert persist.call_count == 2


@pytest.mark.parametrize('device_type,channel_type,supported', [('tizi', 'staging', True), ('tici', 'staging', False), ('tici', 'tici', True)])
@pytest.mark.parametrize('diagnostic_fails', [False, True])
def test_status_loop_keeps_publishing_with_pending_statistics(monkeypatch, device_type, channel_type, supported, diagnostic_fails):
  """Exercise the real loop with a stalled statistics writer and changing hardware.

  Deferred writes remain queued throughout the run. Synchronous statistics would
  stall it; thermal, ignition, and unsupported-device gates must still operate.
  """
  end_event = threading.Event()
  published = []
  pending = []
  now = [100.]
  panda = log.PandaState.new_message(ignitionLine=True, pandaType='tres', harnessStatus='normal')
  peripheral = log.PeripheralState.new_message(pandaType='tres', voltage=14000)
  selfdrive = log.SelfdriveState.new_message(enabled=False)
  signals = {'pandaStates': [panda], 'peripheralState': peripheral, 'selfdriveState': selfdrive,
             'chestnutState': log.ChestnutState.new_message()}

  class SubMaster:
    frame = 0
    updated = {'pandaStates': True, 'selfdriveState': True}
    valid = {'chestnutState': False}
    alive = {'chestnutState': False, 'gpsLocationExternal': False}

    def update(self, _timeout):
      self.frame += 5
      now[0] += .5
      panda.ignitionLine = len(published) != 121

    def __getitem__(self, name):
      return signals[name]

  def thermal_message():
    # Escalate across both thermal bands after the ignition edge, then recover.
    temp = 120. if 125 <= len(published) < 127 else 40.
    return log.DeviceState.new_message(cpuTempC=[temp], gpuTempC=[temp], pmicTempC=[temp], memoryTempC=temp)

  def publish(service, msg):
    assert service == 'deviceState'
    published.append({'started': msg.deviceState.started, 'thermal': str(msg.deviceState.thermalStatus),
                      'som_power': msg.deviceState.somPowerDrawW})
    if len(published) == 130:
      end_event.set()

  def put(key, value, block=False):
    if key in ('UptimeOffroad', 'UptimeOnroad'):
      assert not block, 'Statistics must not wait for a stalled storage writer'
      pending.append((key, value))

  values = {'UptimeOffroad': 10., 'UptimeOnroad': 20., 'HasAcceptedTerms': hardwared.terms_version,
            'HasAcceptedTermsSP': hardwared.terms_version_sp, 'CompletedTrainingVersion': hardwared.training_version}
  params = Mock()
  params.get.side_effect = lambda key, **_kw: values.get(key)
  params.get_bool.return_value = False
  params.put.side_effect = put
  hardware = Mock()
  hardware.get_device_type.return_value = device_type
  hardware.get_thermal_config.return_value.get_msg.side_effect = thermal_message
  hardware.get_gpu_usage_percent.return_value = 0.
  hardware.get_screen_brightness.return_value = 50.
  hardware.get_current_power_draw.return_value = 5.
  sensor_entered, sensor_release = threading.Event(), threading.Event()

  def blocked_som_read():
    sensor_entered.set()
    assert sensor_release.wait(5.), 'test cleanup failed to release sensor'
    return 3.

  hardware.get_som_power_draw.side_effect = blocked_som_read
  hardware.booted.return_value = True
  power = Mock()
  power.get_power_used.return_value = 0
  power.get_car_battery_capacity.return_value = 1_000_000
  power.should_shutdown.return_value = False
  stats = Mock()
  stats.memory_usage_percent.return_value = 20
  stats.cpu_usage_percent.return_value = [10]
  metadata = Mock(return_value=SimpleNamespace(channel='test-channel', channel_type=channel_type))
  persist = Mock()
  for name, value in {'Params': Mock(return_value=params), 'HARDWARE': hardware, 'COMMA_HARDWARE': True,
                      'PowerMonitoring': Mock(return_value=power), 'LinuxSystemStats': Mock(return_value=stats),
                      'Chestnut': Mock(), 'ChestnutStatus': Mock(), 'get_available_percent': lambda **_kw: 90.,
                      'get_short_branch': lambda: 'test-channel', 'get_build_metadata': metadata,
                      'set_usb_state': Mock(), 'set_offroad_alert': persist, 'statlog': Mock(), 'cloudlog': Mock(),
                      'FirstOrderFilter': lambda *_args, **_kw: SimpleNamespace(update=lambda x: x)}.items():
    monkeypatch.setattr(hardwared, name, value)
  monkeypatch.setattr(hardwared.messaging, 'SubMaster', lambda *_args, **_kw: SubMaster())
  monkeypatch.setattr(hardwared.messaging, 'PubMaster', lambda *_args: SimpleNamespace(send=publish))
  monkeypatch.setattr(hardwared.time, 'monotonic', lambda: now[0])
  # Do not touch /dev/kmsg in the test.
  monkeypatch.setattr(hardwared, 'open', Mock(side_effect=OSError()), raising=False)

  diagnostic = Mock()
  if diagnostic_fails:
    diagnostic.published.side_effect = RuntimeError('diagnostic unavailable')
  telemetry = SomPowerTelemetry(hardware.get_som_power_draw, clock=lambda: now[0])
  reader = threading.Thread(target=telemetry.run, args=(end_event,), daemon=True)
  reader.start()
  try:
    assert sensor_entered.wait(2.)
    hardwared.hardware_thread(end_event, queue.Queue(), diagnostic, telemetry)
  finally:
    end_event.set()
    sensor_release.set()
    reader.join(2.)
  assert not reader.is_alive()
  hardware.get_som_power_draw.assert_called_once()
  assert all(p['som_power'] == 0. for p in published)
  assert not any(c.args[0] == 'som_power_draw' for c in hardwared.statlog.sample.call_args_list)

  assert len(published) == 130
  assert diagnostic.published.call_count == len(published)
  assert len(pending) == 4  # Both uptime counters, at count 0 and 120.
  assert all(value >= 0 for _key, value in pending)
  assert pending[2][1] >= pending[0][1]
  assert pending[3][1] >= pending[1][1]
  metadata.assert_called_once()
  support_calls = [c for c in persist.call_args_list if c.args[0] == 'Offroad_TiciSupport']
  assert len(support_calls) == 1
  assert support_calls[0].args[1] == (not supported)
  assert published[1]['started'] == supported  # Initial onroad-cycle delay has elapsed.
  assert not published[121]['started']  # Ignition still takes the device offroad.
  assert not published[126]['started']  # Critical temperature still takes it offroad.
  assert published[129]['started'] == supported
  assert any(p['thermal'] == 'critical' for p in published)


@pytest.mark.parametrize('failure', [None, 'poll', 'log'])
def test_diagnostic_outcomes_do_not_stop_supervision(monkeypatch, failure):
  made_threads = []
  sleeps = []

  def thread(*args, **kwargs):
    item = Mock(ident=42)
    item.optional = kwargs.get('daemon', False)
    item.is_alive.side_effect = lambda: len(sleeps) < 2
    made_threads.append(item)
    return item

  monitor = Mock()
  logger = Mock()
  if failure == 'poll':
    monitor.poll.side_effect = RuntimeError('diagnostic unavailable')
  else:
    monitor.poll.return_value = {'event': 'hardwareStatusStalled'}
    if failure == 'log':
      logger.event.side_effect = RuntimeError('logging unavailable')
  monkeypatch.setattr(hardwared.threading, 'Thread', thread)
  monkeypatch.setattr(hardwared.time, 'sleep', lambda _: sleeps.append(True))
  monkeypatch.setattr(hardwared, 'COMMA_HARDWARE', False)
  monkeypatch.setattr(hardwared, 'HardwareLoopDiagnostics', lambda: monitor)
  monkeypatch.setattr(hardwared, 'cloudlog', logger)
  hardwared.main()
  if failure is None:
    logger.event.assert_called_once_with('hardwareStatusStalled', error=True)
  assert len(sleeps) == 2
  assert len(made_threads) == 3
  for item in made_threads:
    item.start.assert_called_once()
    if item.optional:
      item.join.assert_not_called()
      item.is_alive.assert_not_called()
    else:
      item.join.assert_called_once()
