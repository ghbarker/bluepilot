from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.ford.values import FordFlags
from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt
from bluepilot.selfdrive.car.bp_card_publisher import publish_car_state_bp
from openpilot.cereal import messaging
from openpilot.selfdrive.ui.bp.onroad.steering_limit_state import ford_limit_display, MAX_FEEDBACK_AGE_NS


NOW = 10_000_000_000


def parsed_feedback(status=2, control=2, flags=FordFlags.CANFD, send=True, bus=0):
  parser = CANParser('ford_lincoln_base_pt', [('Lane_Assist_Data3_FD1', 33)], 0)
  packer = CANPacker('ford_lincoln_base_pt')
  frame = packer.make_can_msg('Lane_Assist_Data3_FD1', bus, {'LatCtlLim_D_Stat': status, 'LatCtlSte_D_Stat': control})
  if send:
    parser.update([[NOW, [frame]]])
  car = SimpleNamespace(CP=SimpleNamespace(flags=flags, openpilotLongitudinalControl=False))
  car.car_state_bp_msg = CarStateExt.update_car_state_bp(car, parser, parser)
  return car, parser


@pytest.mark.parametrize('status,label,rgb', [
  (0, 'DEMAND / CAPACITY UNKNOWN', (230, 230, 230)),
  (1, 'NEAR LIMIT', (255, 200, 0)),
  (2, 'LIMIT REACHED', (255, 65, 65)),
  (3, 'LIMIT / DRIVER INPUT', (255, 65, 65)),
])
def test_packed_can_to_publisher_to_display(status, label, rgb):
  car, parser = parsed_feedback(status)
  before = {name: values.copy() for name, values in parser.vl.items()}
  pm = Mock()
  publish_car_state_bp(SimpleNamespace(CS=car), pm, True)
  topic, sent = pm.send.call_args.args
  assert topic == 'carStateBP' and sent.valid
  with messaging.log.Event.from_bytes(sent.to_bytes()) as received:
    feedback = received.carStateBP.fordSteeringLimit
    assert feedback.dataAvailable and feedback.sourceMonoTime == NOW
    assert feedback.status == status and feedback.controlStatus == 2
    display = ford_limit_display(feedback, NOW, True, received.valid)
    assert (display.label, display.rgb) == (label, rgb)
  assert not hasattr(car, 'lat_ctl_lim_stat')  # no new input to dormant controller branches
  assert before == dict(parser.vl)


@pytest.mark.parametrize('kwargs', [{'send': False}, {'bus': 1}, {'flags': 0}])
def test_missing_unsupported_or_wrong_bus_never_becomes_spare_capacity(kwargs):
  car, _ = parsed_feedback(**kwargs)
  feedback = car.car_state_bp_msg.carStateBP.fordSteeringLimit
  assert not feedback.dataAvailable
  assert ford_limit_display(feedback, NOW, True, True).label == 'DEMAND / CAPACITY UNKNOWN'


@pytest.mark.parametrize('control', [0, 1, 3, 4, 5, 6, 7])
def test_no_active_pscm_control_means_unknown(control):
  car, _ = parsed_feedback(control=control)
  assert ford_limit_display(car.car_state_bp_msg.carStateBP.fordSteeringLimit, NOW, True, True).rgb == (230, 230, 230)


def test_republishing_cannot_refresh_stale_can_or_invalid_transport():
  car, parser = parsed_feedback()
  parser.update([[NOW + MAX_FEEDBACK_AGE_NS + 1, []]])
  car.car_state_bp_msg = CarStateExt.update_car_state_bp(car, parser, parser)
  feedback = car.car_state_bp_msg.carStateBP.fordSteeringLimit
  assert feedback.sourceMonoTime == NOW
  assert ford_limit_display(feedback, NOW + MAX_FEEDBACK_AGE_NS, True, True).label == 'LIMIT REACHED'
  assert ford_limit_display(feedback, NOW + MAX_FEEDBACK_AGE_NS + 1, True, True).rgb == (230, 230, 230)
  assert ford_limit_display(feedback, NOW - 1, True, True).rgb == (230, 230, 230)
  assert ford_limit_display(feedback, NOW, False, True).label == ''
  pm = Mock()
  publish_car_state_bp(SimpleNamespace(CS=car), pm, False)
  sent = pm.send.call_args.args[1]
  assert not sent.valid
  assert ford_limit_display(sent.carStateBP.fordSteeringLimit, NOW, True, sent.valid).rgb == (230, 230, 230)


def test_old_recording_without_new_field_is_unknown():
  feedback = messaging.new_message('carStateBP').carStateBP.fordSteeringLimit
  assert ford_limit_display(feedback, NOW, True, True).label == 'DEMAND / CAPACITY UNKNOWN'
