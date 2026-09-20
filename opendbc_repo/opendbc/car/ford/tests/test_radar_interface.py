import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.radar_interface import RadarInterface
from opendbc.car.ford.values import CAR, RADAR


@pytest.fixture
def radar():
  cp = CarInterface.get_non_essential_params(CAR.FORD_MUSTANG_MACH_E_MK1)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, cp.carFingerprint)
  return RadarInterface(cp, cp_sp), CANPacker(RADAR.STEER_ASSIST_DATA), CanBus(cp).camera


def update(radar, confidence, distance=40., speed=-1.5, lateral=-0.5):
  interface, packer, bus = radar
  frame = packer.make_can_msg("Steer_Assist_Data", bus, {
    "CmbbObjConfdnc_D_Stat": confidence,
    "CmbbObjDistLong_L_Actl": distance,
    "CmbbObjDistLat_L_Actl": lateral,
    "CmbbObjRelLong_V_Actl": speed,
    "CmbbObjRelLat_V_Actl": 0.2,
  })
  result = interface.update([0, [frame]])
  assert result is not None
  # Exercise the schema and serialization used by card, not just the parser values.
  with structs.RadarData.from_bytes(result.to_bytes()) as decoded:
    assert not decoded.errors.canError
    return decoded.to_dict()


@pytest.mark.parametrize("confidence", [1, 2, 3])
def test_mach_e_first_lead_serializes(radar, confidence):
  result = update(radar, confidence)
  assert len(result["points"]) == 1
  point = result["points"][0]
  assert point["dRel"] == pytest.approx(40.)
  assert point["yRel"] == pytest.approx(-0.5)
  assert point["vRel"] == pytest.approx(-1.5)


def test_mach_e_lead_tracking_loss_and_reacquisition(radar):
  assert update(radar, 0)["points"] == []
  first = update(radar, 3)["points"][0]
  next_point = update(radar, 3, distance=39.9)["points"][0]
  assert next_point["trackId"] == first["trackId"]
  assert next_point["dRel"] == pytest.approx(39.9)
  assert update(radar, 0)["points"] == []
  assert radar[0].vRelCol == {}
  reacquired = update(radar, 1)["points"][0]
  assert reacquired["trackId"] > first["trackId"]
  assert reacquired["vRel"] == pytest.approx(-1.5)


def test_mach_e_zero_speed_fallback_and_discontinuous_lead(radar):
  first = update(radar, 3, speed=0.)["points"][0]
  second = update(radar, 3, distance=40.1, speed=0.)["points"][0]
  assert second["trackId"] == first["trackId"]
  assert second["vRel"] == pytest.approx(0.1, abs=1e-5)
  third = update(radar, 3, distance=40.2, speed=0.)["points"][0]
  assert third["vRel"] == pytest.approx(0.2, abs=1e-5)
  measured = update(radar, 3, distance=40.2, speed=-1.5)["points"][0]
  assert measured["vRel"] == pytest.approx(-1.5)
  assert len(radar[0].vRelCol[0]) == 0
  replacement = update(radar, 3, distance=60., speed=-1.5)["points"][0]
  assert replacement["trackId"] > first["trackId"]


def test_mach_e_ignores_lead_on_wrong_bus(radar):
  interface, packer, camera_bus = radar
  frame = packer.make_can_msg("Steer_Assist_Data", camera_bus + 1, {"CmbbObjConfdnc_D_Stat": 3})
  assert interface.update([0, [frame]]) is None
  assert interface.pts == {}
