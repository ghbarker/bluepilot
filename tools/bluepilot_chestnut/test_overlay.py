import math
from unittest.mock import patch

import pytest

from opendbc.car import structs, Bus
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP
from opendbc.sunnypilot.car.ford.chestnut_compat import limit_curvature_for_chestnut
from opendbc.safety.tests import test_ford_bluepilot as ford_safety
from openpilot.common.params import Params


@pytest.mark.parametrize('platform', list(CAR))
def test_ford_interface_selects_private_safety(platform):
  cp = CarInterface.get_non_essential_params(platform)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, platform)
  assert cp_sp.safetyParam & FordSafetyFlagsSP.BLUEPILOT
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  assert convert_to_capnp(cp_sp).safetyParam == cp_sp.safetyParam
  assert cp.alphaLongitudinalAvailable
  assert not cp.openpilotLongitudinalControl


@pytest.mark.parametrize('safety_class', [ford_safety.TestFordLongitudinalSafety, ford_safety.TestFordCANFDStockSafety])
def test_current_curvature_envelope_passes_firmware(safety_class):
  # Exercise the Python integration against actual compiled Panda hooks. Include
  # steering reversals and measured curvature outside the command-error band.
  t = safety_class()
  for speed in (5., 11., 15., 25., 35.):
    t.setUp()
    t.safety.set_controls_allowed(True)
    previous = 0.
    for frame in range(500):
      measured = 0.001 * math.sin(frame / 30.)
      requested = 0.02 * math.sin(frame / 25.)
      t._reset_curvature_measurement(-measured, speed)
      limited = limit_curvature_for_chestnut(requested, previous, measured, speed, True)
      assert t._tx(t._lat_ctl_msg(True, 0, 0, -limited, 0)), (speed, frame, previous, measured, requested, limited)
      previous = limited
    # Neutral inactive frame releases control and resets the next command origin.
    assert t._tx(t._lat_ctl_msg(False, 0, 0, 0, 0))
    assert limit_curvature_for_chestnut(0.02, previous, measured, speed, False) == 0.


@pytest.mark.parametrize('platform,safety_class', [
  (CAR.FORD_EXPLORER_MK6, ford_safety.TestFordLongitudinalSafety),
  (CAR.FORD_MUSTANG_MACH_E_MK1, ford_safety.TestFordCANFDStockSafety),
])
def test_controller_runs_current_interfaces_and_mode_changes(platform, safety_class, tmp_path):
  params = Params(str(tmp_path / 'params'))
  cp = CarInterface.get_non_essential_params(platform)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, platform)
  with patch('opendbc.car.ford.carcontroller.Params', return_value=params):
    ci = CarInterface(cp, cp_sp)
  cc = structs.CarControl()
  cc_sp = structs.CarControlSP()
  cc.latActive = True
  cc.actuators.curvature = 0.001
  ci.CS.out = structs.CarState()
  ci.CS.out.vEgo = ci.CS.out.vEgoRaw = 15.
  ci.CS.buttons_stock_values = {}
  ci.CS.acc_tja_status_stock_values = ci.can_parsers[Bus.cam].vl['ACCDATA_3']
  ci.CS.lkas_status_stock_values = ci.can_parsers[Bus.cam].vl['IPMA_Data']
  safety = safety_class()
  safety.setUp()
  safety.safety.set_controls_allowed(True)
  for mode, bypass in ((0, False), (1, False), (0, False), (0, True), (1, False)):
    params.put('FordPrefLateralControl', mode, block=True)
    params.put_bool('disable_BP_lat_UI', bypass, block=True)
    first_lateral_message = True
    for _ in range(30):
      safety._reset_curvature_measurement(0., 15.)
      safety.safety.set_timer(ci.CC.frame * 10000)
      acts, messages = ci.apply(cc.as_reader(), cc_sp, ci.CC.frame * 10_000_000)
      assert math.isfinite(acts.curvature)
      if ci.CC.frame % CarControllerParams.STEER_STEP == 1:
        assert any(msg[0] in (0x3d3, 0x3d6) for msg in messages)
      for addr, data, bus in messages:
        if addr in (0x3d3, 0x3d6):
          enabled = (data[0] >> 4) & 7 if addr == 0x3d6 else (data[4] >> 2) & 7
          assert bool(enabled) != first_lateral_message, (mode, bypass, ci.CC.frame)
          first_lateral_message = False
        packet = ford_safety.libsafety_py.make_CANPacket(addr, bus, data)
        assert safety._tx(packet), (platform, mode, bypass, ci.CC.frame, hex(addr), data.hex())

def test_bluepilot_messages_use_current_schema(tmp_path):
  from bluepilot.selfdrive.car.bp_card_publisher import publish_controller_state_bp, publish_car_state_bp
  from openpilot.cereal import messaging, log
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  from types import SimpleNamespace

  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPLORER_MK6)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.FORD_EXPLORER_MK6)
  assert convert_to_capnp(cp_sp).safetyParam & FordSafetyFlagsSP.BLUEPILOT
  state = messaging.new_message('carStateBP')
  ci = SimpleNamespace(CP=cp, CC=SimpleNamespace(lateralUncertainty=0.4, primary_lateral_control=1,
                                               disable_BP_lat_UI=False), CS=SimpleNamespace(car_state_bp_msg=state))
  messages = {}
  class Publisher:
    def send(self, service, event):
      messages[service] = event.to_bytes()
  publisher = Publisher()
  params = Params(str(tmp_path / 'params'))
  with patch('bluepilot.selfdrive.car.bp_card_publisher.Params', return_value=params):
    publish_controller_state_bp(ci, publisher)
    publish_car_state_bp(ci, publisher, True)
  with log.Event.from_bytes(messages['controllerStateBP']) as event:
    assert event.valid
    assert str(event.controllerStateBP.activeLateralMode) == 'angle'
    assert event.controllerStateBP.bmsFingerprint == CAR.FORD_EXPLORER_MK6
  with log.Event.from_bytes(messages['carStateBP']) as event:
    assert event.valid


def test_portal_typed_params_and_categories(tmp_path):
  from bluepilot.backend.params.params_manager import set_param_value, get_all_params, get_params_by_category
  params = Params(str(tmp_path / 'params'))
  assert set_param_value('FordPrefLateralControl', '1', params)['success']
  assert set_param_value('FordHighSpeedDampening_ang', '0.75', params)['success']
  assert params.get('FordPrefLateralControl') == 1
  assert params.get('FordHighSpeedDampening_ang') == 0.75
  assert get_all_params(params)['FordPrefLateralControl']['value'] == 1
  assert 'System' in get_params_by_category(params)


@pytest.mark.parametrize('metric', [True, False])
def test_cruise_fallback_and_initial_increase_use_cluster_units(metric):
  from openpilot.cereal import custom
  from openpilot.common.constants import CV
  from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement, State
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPLORER_MK6)
  controller = IntelligentCruiseButtonManagement(cp, CarInterface.get_non_essential_params_sp(cp, cp.carFingerprint))
  controller.is_metric = metric
  state = structs.CarState()
  state.vEgo = 60 * CV.MPH_TO_MS
  state.cruiseState.available = state.cruiseState.enabled = True
  state.cruiseState.speedCluster = state.vEgo
  plan = custom.LongitudinalPlanSP.new_message(vTarget=0.)
  controller.update_calculations(state, plan)
  expected = round(60 * CV.MPH_TO_MS * CV.MS_TO_KPH) if metric else 60
  assert controller.v_target == expected
  controller.is_ready = True
  controller.state = State.preActive
  controller.v_target = expected + 6
  controller.update_state_machine()
  assert controller.state == State.holding


def test_portal_serves_frontend_and_current_params(tmp_path):
  import http.client
  import json
  import threading
  from http.server import ThreadingHTTPServer
  from bluepilot.backend import bp_portal

  params = Params(str(tmp_path / 'params'))
  params.put('FordPrefLateralControl', 1, block=True)
  with patch.object(bp_portal, 'params', params), patch.object(bp_portal, 'should_server_run', return_value=True), \
       patch.object(bp_portal, 'is_onroad', return_value=False):
    server = ThreadingHTTPServer(('127.0.0.1', 0), bp_portal.WebRoutesHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = http.client.HTTPConnection(*server.server_address, timeout=5)
    try:
      client.request('GET', '/')
      response = client.getresponse()
      assert response.status == 200
      assert b'<html' in response.read().lower()
      client.request('GET', '/api/params')
      response = client.getresponse()
      assert response.status == 200
      payload = json.loads(response.read())
      assert payload['success']
      assert payload['params']['FordPrefLateralControl']['value'] == 1
    finally:
      client.close()
      server.shutdown()
      server.server_close()
      thread.join(timeout=5)
