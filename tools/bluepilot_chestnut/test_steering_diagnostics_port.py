"""Verify the logged steering request against real CAN packing and native schema."""
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford import fordcan_ext
from opendbc.sunnypilot.car.ford.steering_diagnostics import steering_command_snapshot, fill_eps_diagnostics
from openpilot.cereal import custom, log
from openpilot.selfdrive.car.helpers import convert_to_capnp


NOW = 10_000_000_000
DBC = "ford_lincoln_base_pt"


@pytest.mark.parametrize("can_fd", [False, True])
@pytest.mark.parametrize("angle,curvature", [(0., 0.), (-.5, -.02), (.5235, .02094), (-.12326, .001234), (.12326, -.001234)])
@pytest.mark.parametrize("active", [False, True])
def test_snapshot_decodes_signed_quantized_final_packet(can_fd, angle, curvature, active):
  packer = CANPacker(DBC)
  bus = SimpleNamespace(main=0)
  if not active:
    angle = curvature = 0.
  if can_fd:
    msg = fordcan_ext.create_lat_ctl2_msg(packer, bus, int(active), 2, 1, 0., angle, curvature, 0., 3)
  else:
    msg = fordcan_ext.create_lat_ctl_msg(packer, bus, active, 2, 1, 0., angle, curvature, 0.)
  original = tuple(msg)
  name = "LateralMotionControl2" if can_fd else "LateralMotionControl"
  parser = CANParser(DBC, [(name, 20)], 0)
  parser.update([(NOW, [msg])])
  snapshot = steering_command_snapshot(msg, 123, NOW)
  assert tuple(msg) == original
  assert snapshot["dataAvailable"] and snapshot["canFd"] == can_fd
  assert snapshot["mode"] == int(active)
  assert snapshot["pathAngle"] == pytest.approx(parser.vl[name]["LatCtlPath_An_Actl"])
  assert snapshot["curvature"] == pytest.approx(parser.vl[name]["LatCtlCurv_No_Actl"])
  assert abs(snapshot["pathAngle"] - angle) <= .000250001
  assert abs(snapshot["curvature"] - curvature) <= .000010001
  state = structs.ControllerStateBP()
  state.fordSteeringCommand = structs.ControllerStateBP.FordSteeringCommand(**snapshot)
  encoded = convert_to_capnp(state).to_bytes()
  with custom.ControllerStateBP.from_bytes(encoded) as decoded:
    assert decoded.fordSteeringCommand.sourceMonoTime == NOW
    assert decoded.fordSteeringCommand.frame == 123
    assert decoded.fordSteeringCommand.pathAngle == pytest.approx(snapshot["pathAngle"])
    assert decoded.fordSteeringCommand.dataAvailable


def test_eps_native_can_and_schema_roundtrip_retains_actual_timestamp():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [("EPAS_INFO", 50)], 0)
  values = {"SteMdule_I_Est": 0., "SteMdule_U_Meas": 12.5, "SteMdule_D_Stat": 2}
  parser.update([(NOW, [packer.make_can_msg("EPAS_INFO", 0, values)])])
  state = custom.CarStateBP.new_message()
  fill_eps_diagnostics(state.fordEps, parser, NOW + 1)
  with custom.CarStateBP.from_bytes(state.to_bytes()) as decoded:
    assert decoded.fordEps.dataAvailable
    assert decoded.fordEps.sourceMonoTime == NOW
    assert decoded.fordEps.estimatedCurrentAmps == 0.
    assert decoded.fordEps.voltage == 12.5
  fill_eps_diagnostics(state.fordEps, parser, NOW + 100_000_001)
  assert not state.fordEps.dataAvailable
  assert state.fordEps.sourceMonoTime == NOW


@pytest.mark.parametrize("platform", [CAR.FORD_EXPLORER_MK6, CAR.FORD_MUSTANG_MACH_E_MK1])
def test_controller_snapshot_persists_and_matches_each_final_packet(platform, tmp_path):
  from opendbc.car.ford.interface import CarInterface
  from openpilot.common.params import Params
  params = Params(str(tmp_path / "params"))
  cp = CarInterface.get_non_essential_params(platform)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, platform)
  with patch("opendbc.car.ford.carcontroller.Params", return_value=params):
    ci = CarInterface(cp, cp_sp)
  cc, cc_sp = structs.CarControl(), structs.CarControlSP()
  cc.latActive = True
  cc.actuators.curvature = .001
  ci.CS.out = structs.CarState(vEgo=15., vEgoRaw=15.)
  ci.CS.buttons_stock_values = {}
  ci.CS.acc_tja_status_stock_values = ci.can_parsers[Bus.cam].vl["ACCDATA_3"]
  ci.CS.lkas_status_stock_values = ci.can_parsers[Bus.cam].vl["IPMA_Data"]
  previous = {"dataAvailable": False}
  for mode, bypass in ((0, False), (1, False), (0, True)):
    params.put("FordPrefLateralControl", mode, block=True)
    params.put_bool("disable_BP_lat_UI", bypass, block=True)
    for tick in range(20):
      cc.latActive = tick < 15
      frame = ci.CC.frame
      now = NOW + frame * 10_000_000
      _, messages = ci.apply(cc.as_reader(), cc_sp, now)
      packets = [message for message in messages if message[0] in (0x3D3, 0x3D6)]
      if packets:
        previous = steering_command_snapshot(packets[-1], frame, now)
        assert previous["dataAvailable"]
        if not cc.latActive:
          assert previous["mode"] == 0 and previous["pathAngle"] == 0. and previous["curvature"] == 0.
      assert ci.CC.fordSteeringCommand == previous
  # Real publisher conversion must preserve the stored timestamp, not stamp a
  # retained command as a fresh sample on every 100 Hz publication.
  from bluepilot.selfdrive.car.bp_card_publisher import publish_controller_state_bp
  events = {}
  publisher = SimpleNamespace(send=lambda name, event: events.update({name: event.to_bytes()}))
  with patch("bluepilot.selfdrive.car.bp_card_publisher.Params", return_value=params):
    publish_controller_state_bp(ci, publisher)
  with log.Event.from_bytes(events["controllerStateBP"]) as event:
    cmd = event.controllerStateBP.fordSteeringCommand
    assert cmd.dataAvailable and cmd.sourceMonoTime == previous["sourceMonoTime"]
    assert cmd.frame == previous["frame"]


def test_eps_capture_does_not_advance_real_parser_can_invalid_debounce():
  packer = CANPacker(DBC)
  parser = CANParser(DBC, [("EPAS_INFO", 50), ("Steering_Data_FD1", 100)], 0)
  values = {"SteMdule_I_Est": 0., "SteMdule_U_Meas": 12.5, "SteMdule_D_Stat": 2}
  parser.update([(NOW, [packer.make_can_msg("EPAS_INFO", 0, values),
                       packer.make_can_msg("Steering_Data_FD1", 0, {})])])
  assert parser.can_valid
  assert parser.can_invalid_cnt == 0
  # EPAS remains fresh but a different required message goes stale. Reading
  # can_valid here would advance the control path's invalid-CAN debounce.
  now = NOW + 2_000_000_000
  parser.update([(now, [packer.make_can_msg("EPAS_INFO", 0, values)])])
  before = parser.can_invalid_cnt
  state = custom.CarStateBP.new_message()
  for _ in range(10):
    fill_eps_diagnostics(state.fordEps, parser, now)
  assert state.fordEps.dataAvailable
  assert parser.can_invalid_cnt == before


@pytest.mark.parametrize("can_valid", [False, True])
def test_publisher_masks_eps_availability_with_existing_control_validity(can_valid):
  from bluepilot.selfdrive.car.bp_card_publisher import publish_car_state_bp
  from openpilot.cereal import messaging
  state = messaging.new_message("carStateBP", valid=True)
  state.carStateBP.fordEps.dataAvailable = True
  state.carStateBP.fordEps.sourceMonoTime = NOW
  state.carStateBP.fordEps.estimatedCurrentAmps = 3.
  ci = SimpleNamespace(CS=SimpleNamespace(car_state_bp_msg=state))
  events = {}
  publisher = SimpleNamespace(send=lambda name, event: events.update({name: event.to_bytes()}))
  publish_car_state_bp(ci, publisher, can_valid)
  with log.Event.from_bytes(events["carStateBP"]) as event:
    assert event.valid == can_valid
    assert event.carStateBP.fordEps.dataAvailable == can_valid
    assert event.carStateBP.fordEps.sourceMonoTime == NOW
