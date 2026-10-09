"""Reconcile a rejected CAN-FD angle request with Panda's retained rate history.

A returned packet means Panda admitted it to CAN transmission, not that the EPS
followed it. Recovery requires exact returned/rejected packet matches. Missing
or ambiguous returns never provide a replacement rate reference.
"""
from collections import deque
from dataclasses import dataclass

from opendbc.sunnypilot.car.ford.steering_diagnostics import can_clock_nanos


MAX_AGE_FRAMES = 25  # 250 ms at the controller's 100 Hz cadence, below counter wrap
MAX_AGE_NS = 250_000_000


@dataclass
class SentAngle:
  data: bytes
  frame: int
  can_time: int
  accepted: bool | None = None
  ambiguous: bool = False


class AngleCommandRecovery:
  def __init__(self, bus: int, clock=can_clock_nanos):
    self.bus = bus
    self.clock = clock
    self.sent: deque[SentAngle] = deque(maxlen=6)
    self.can_time = 0
    self.recovered_frame = -1

  def reset(self):
    self.sent.clear()
    self.recovered_frame = -1

  def record(self, message, frame: int):
    address, data, bus = message
    if address != 0x3D6 or bus != self.bus or len(data) != 8:
      self.reset()
      return
    if self.sent and not 0 < frame - self.sent[-1].frame <= MAX_AGE_FRAMES:
      self.reset()
    self.sent.append(SentAngle(bytes(data), frame, self.can_time))

  def update(self, can_packets, frame: int):
    for timestamp, messages in can_packets:
      # Use the CAN clock on both sides of this comparison. Python's monotonic
      # clock excludes suspend on Linux; CAN event timestamps include it.
      if timestamp <= self.can_time:
        continue
      self.can_time = timestamp
      if not self.sent:
        continue
      for address, data, source in messages:
        if address != 0x3D6 or source not in (self.bus + 128, self.bus + 192):
          continue
        matches = [sent for sent in self.sent if sent.data == data and sent.can_time > 0
                   and 0 < timestamp - sent.can_time <= MAX_AGE_NS and 0 <= frame - sent.frame <= MAX_AGE_FRAMES]
        if len(matches) != 1:
          continue
        sent = matches[0]
        accepted = source == self.bus + 128
        if sent.accepted is not None and sent.accepted != accepted:
          sent.ambiguous = True
        sent.accepted = accepted

  def recover(self, frame: int) -> float | None:
    if not self.sent:
      return None
    newest = self.sent[-1]
    if (newest.accepted is not False or newest.ambiguous or newest.frame == self.recovered_frame
        or ((newest.data[0] >> 4) & 7) != 1):
      return None
    now_nanos = self.clock()
    for sent in reversed(self.sent):
      # Received timestamps and frame counters can both freeze during a stall.
      # The current BOOTTIME clock keeps expiry effective even on update([]).
      if (not 0 <= frame - sent.frame <= MAX_AGE_FRAMES or not 0 <= now_nanos - sent.can_time <= MAX_AGE_NS
          or sent.ambiguous or sent.accepted is None):
        return None
      if sent.accepted:
        self.recovered_frame = newest.frame
        raw_angle = ((sent.data[3] & 0x1F) << 6) | (sent.data[4] >> 2)
        # Controller angle has the opposite sign to the quantized CAN field.
        return -(raw_angle - 1000) * 0.0005
    return None
