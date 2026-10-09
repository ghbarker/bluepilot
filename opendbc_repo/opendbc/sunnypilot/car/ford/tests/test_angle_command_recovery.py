import pytest

from opendbc.sunnypilot.car.ford.angle_command_recovery import AngleCommandRecovery, MAX_AGE_FRAMES, MAX_AGE_NS


def packet(raw_angle=1000, counter=0, mode=1, bus=0):
  data = bytearray(8)
  data[0] = mode << 4
  data[3] = raw_angle >> 6
  data[4] = (raw_angle & 63) << 2
  data[7] = counter
  return 0x3D6, bytes(data), bus


def returned(message, accepted=True):
  address, data, bus = message
  return address, data, bus + (128 if accepted else 192)


def new_tracker(bus=0):
  tracker = AngleCommandRecovery(bus, clock=lambda: tracker.can_time)
  return tracker


def seed(bus=0):
  tracker = new_tracker(bus)
  tracker.update([(100, [])], 0)
  first = packet(1100, bus=bus)
  tracker.record(first, 0)
  tracker.update([(110, [returned(first)])], 1)
  return tracker, first


@pytest.mark.parametrize('bus', [0, 4])
@pytest.mark.parametrize('raw_angle', [600, 999, 1000, 1001, 1400])
def test_recover_only_after_matched_rejection(bus, raw_angle):
  tracker = new_tracker(bus)
  tracker.update([(100, [])], 0)
  first = packet(raw_angle, bus=bus)
  tracker.record(first, 0)
  tracker.update([(110, [returned(first)])], 1)
  assert tracker.recover(5) is None  # Ordinary accepted traffic is untouched.
  rejected = packet(raw_angle + 18, counter=1, bus=bus)
  tracker.record(rejected, 5)
  assert tracker.recover(5) is None
  tracker.update([(120, [returned(rejected, False)])], 6)
  assert tracker.recover(10) == pytest.approx(-(raw_angle - 1000) * .0005)
  assert tracker.recover(10) is None  # A rejection is consumed once.


@pytest.mark.parametrize('count', [1, 2, 3, 4])
def test_consecutive_rejections_use_last_admitted_request(count):
  tracker, _ = seed()
  for index in range(1, count + 1):
    msg = packet(1100 + index * 18, counter=index)
    tracker.record(msg, index * 5)
    tracker.update([(110 + index * 10, [returned(msg, False)])], index * 5 + 1)
  assert tracker.recover(count * 5 + 2) == pytest.approx(-.05)


def test_missing_intervening_return_is_not_treated_as_rejection():
  tracker, _ = seed()
  tracker.record(packet(1118, counter=1), 5)
  rejected = packet(1136, counter=2)
  tracker.record(rejected, 10)
  tracker.update([(120, [returned(rejected, False)])], 11)
  assert tracker.recover(15) is None


def test_newer_accepted_packet_supersedes_earlier_rejection():
  tracker, _ = seed()
  rejected, accepted = packet(1118, counter=1), packet(1105, counter=2)
  tracker.record(rejected, 5)
  tracker.record(accepted, 10)
  tracker.update([(120, [returned(accepted), returned(rejected, False)])], 11)
  assert tracker.recover(15) is None


def test_out_of_order_returns_are_resolved_in_send_order():
  tracker, _ = seed()
  accepted, rejected = packet(1118, counter=1), packet(1136, counter=2)
  tracker.record(accepted, 5)
  tracker.record(rejected, 10)
  tracker.update([(120, [returned(rejected, False)])], 11)
  assert tracker.recover(11) is None
  tracker.update([(130, [returned(accepted)])], 12)
  assert tracker.recover(15) == pytest.approx(-.059)


@pytest.mark.parametrize('case', ['wrong_bus', 'wrong_payload', 'wrong_address', 'ordinary_rx', 'old_clock', 'old_frame', 'future_frame'])
def test_unrelated_or_stale_return_cannot_recover(case):
  tracker, _ = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  reply = returned(rejected, False)
  clock, frame = 120, 6
  if case == 'wrong_bus':
    reply = reply[:2] + (reply[2] + 1,)
  if case == 'wrong_payload':
    reply = returned(packet(1117, counter=1), False)
  if case == 'wrong_address':
    reply = (0x3D3,) + reply[1:]
  if case == 'ordinary_rx':
    reply = rejected
  if case == 'old_clock':
    clock = 110
  if case == 'old_frame':
    frame = 5 + MAX_AGE_FRAMES + 1
  if case == 'future_frame':
    frame = 4
  tracker.update([(clock, [reply])], frame)
  assert tracker.recover(10) is None


def test_missing_or_expired_accepted_reference_cannot_recover():
  tracker = new_tracker()
  tracker.update([(100, [])], 0)
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  assert tracker.recover(10) is None
  tracker, _ = seed()
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  assert tracker.recover(MAX_AGE_FRAMES + 1) is None


def test_mode_reset_discards_previous_rejections():
  tracker, _ = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  tracker.reset()
  assert tracker.recover(10) is None


def test_inactive_request_does_not_restore_nonzero_reference():
  tracker, _ = seed()
  rejected = packet(mode=0, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  assert tracker.recover(10) is None


def test_confirmed_neutral_can_seed_next_active_recovery():
  tracker = new_tracker()
  tracker.update([(100, [])], 0)
  inactive = packet(mode=0)
  tracker.record(inactive, 0)
  tracker.update([(110, [returned(inactive)])], 1)
  rejected = packet(1018, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  assert tracker.recover(10) == 0.


def test_conflicting_returns_and_duplicate_pending_payloads_are_ambiguous():
  tracker, first = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(first, False), returned(rejected, False)])], 6)
  assert tracker.recover(10) is None
  tracker, _ = seed()
  tracker.record(rejected, 5)
  tracker.record(rejected, 10)
  tracker.update([(120, [returned(rejected, False)])], 11)
  assert tracker.recover(15) is None


@pytest.mark.parametrize('frame', [0, 100])
def test_controller_time_reset_or_long_gap_discards_reference(frame):
  tracker, _ = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, frame)
  tracker.update([(120, [returned(rejected, False)])], frame + 1)
  assert tracker.recover(frame + 5) is None


def test_no_can_clock_anchor_and_repeated_normal_returns_do_not_change_commands():
  tracker = new_tracker()
  first = packet()
  tracker.record(first, 0)
  tracker.update([(100, [returned(first)])], 1)
  assert tracker.sent[0].accepted is None
  for index in range(1, 65):
    msg = packet(1000 + index, counter=index % 16)
    tracker.record(msg, index * 5)
    tracker.update([(100 + index * 10, [returned(msg), returned(msg)])], index * 5 + 1)
    assert tracker.recover(index * 5 + 2) is None
  assert len(tracker.sent) == 6


@pytest.mark.parametrize('late_return', [False, True])
def test_real_time_stall_expires_reference_even_if_controller_frame_stalls(late_return):
  tracker, _ = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120 + (MAX_AGE_NS if late_return else 0), [returned(rejected, False)])], 6)
  tracker.update([(130 + MAX_AGE_NS, [])], 7)
  assert tracker.recover(10) is None


@pytest.mark.parametrize('elapsed_ns', [MAX_AGE_NS + 1, 5_000_000_000, 60_000_000_000])
def test_empty_can_update_after_stall_cannot_reuse_frozen_freshness(elapsed_ns):
  tracker, _ = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  tracker.clock = lambda: 120 + elapsed_ns
  tracker.update([], 7)
  assert tracker.recover(10) is None


def test_future_clock_anchor_cannot_be_used():
  tracker, _ = seed()
  rejected = packet(1118, counter=1)
  tracker.record(rejected, 5)
  tracker.update([(120, [returned(rejected, False)])], 6)
  tracker.clock = lambda: 99
  assert tracker.recover(10) is None
