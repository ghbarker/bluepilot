from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from opendbc.car.fingerprints import MIGRATION
from opendbc.car.ford.values import CAR
from opendbc.car.structs import CarParams
from openpilot.cereal import log
from openpilot.tools import ford_lmc_safety_replay as replay, ford_pinion_replay as pinion, ford_yaw_health_check as yaw


def test_edge_legacy_identity_and_menu():
  from opendbc.sunnypilot.car.platform_list import get_car_list
  assert MIGRATION['FORD EDGE 2ND GEN'] == CAR.FORD_EDGE_MK2
  entry = get_car_list()['Ford Edge 2019-24']
  assert entry['platform'] == CAR.FORD_EDGE_MK2
  assert entry['year'] == [str(year) for year in range(2019, 2025)]


def replay_messages(include_sp=True):
  event = log.Event.new_message()
  cp = event.init('carParams')
  cp.brand = 'ford'
  cp.init('safetyConfigs', 1)
  cp.safetyConfigs[0].safetyModel = 'ford'
  cp.safetyConfigs[0].safetyParam = 2  # CAN FD
  tx = log.Event.new_message()
  tx.init('sendcan', 1)
  tx.sendcan[0].address = 0x3d6
  tx.sendcan[0].dat = bytes(8)
  messages = [event, tx]
  if include_sp:
    sp = log.Event.new_message()
    sp.init('carParamsSP').safetyParam = 7
    messages.append(sp)
  return messages


def test_replay_uses_current_compiled_runner_and_preserves_recorded_flags(monkeypatch):
  from opendbc.safety.tests.safety_replay import replay_drive
  messages = replay_messages()
  monkeypatch.setattr(replay, 'LogReader', lambda _: messages)
  runner = Mock(return_value=False)
  monkeypatch.setattr(replay_drive, 'replay_drive', runner)
  assert not replay.run_current_route('test route')
  assert runner.call_args.args == (messages, int(CarParams.SafetyModel.ford), 2, 0, 0x4007)


def test_replay_requires_missing_safety_metadata(monkeypatch):
  monkeypatch.setattr(replay, 'LogReader', lambda _: replay_messages(False))
  with pytest.raises(ValueError, match='Missing carParamsSP'):
    replay.run_current_route('test route')


def test_empty_logs_cannot_produce_a_replay_pass(monkeypatch):
  monkeypatch.setattr(replay, 'LogReader', lambda _: [])
  with pytest.raises(ValueError):
    replay.run_current_route('empty route')


def test_explorer_diagnostic_rejects_mach_e_geometry(monkeypatch):
  event = log.Event.new_message()
  cp = event.init('carParams')
  cp.carFingerprint = str(CAR.FORD_MUSTANG_MACH_E_MK1)
  cp.flags = 1
  monkeypatch.setattr(pinion, 'LogReader', lambda _: [event])
  with pytest.raises(ValueError, match='Explorer CAN only'):
    pinion.run_route('test route', [0])


def test_yaw_health_is_inconclusive_without_independent_imu(monkeypatch, capsys):
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.vehicle_model import VehicleModel
  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  vm = VehicleModel(cp)
  messages = [SimpleNamespace(which=lambda: 'carParams', carParams=cp)]
  for angle in np.sin(np.arange(600) / 20) * 0.1:
    state = SimpleNamespace(vEgo=15., steeringAngleDeg=np.degrees(angle), yawRate=vm.calc_curvature(angle, 15., 0.) * 15.)
    messages.append(SimpleNamespace(which=lambda: 'carState', carState=state))
  monkeypatch.setattr(yaw, 'LogReader', lambda _: messages)
  assert yaw.run('synthetic route without IMU') == 2
  assert 'INCONCLUSIVE' in capsys.readouterr().out
