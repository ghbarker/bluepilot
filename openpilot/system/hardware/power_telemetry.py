"""Keep optional SoM power telemetry outside thermal and ignition monitoring."""
import math
import time


class SomPowerTelemetry:
  SAMPLE_INTERVAL = 0.5
  MAX_SAMPLE_AGE = 2.0

  def __init__(self, read_power, clock=time.monotonic):
    self.read_power = read_power
    self.clock = clock
    self.latest = None

  def sample(self):
    # Voltage is read before current. A slow current read must not make the
    # resulting product, which contains old voltage, appear newly measured.
    started = self.clock()
    try:
      value = float(self.read_power())
      # The hardware reader already returns zero when a sysfs read fails or the
      # sensor is unavailable. Preserve signed nonzero readings as reported.
      self.latest = (started, value) if math.isfinite(value) and value != 0. else None
    except Exception:
      self.latest = None

  def get(self):
    # A tuple assignment/read is atomic under CPython; never wait on the reader.
    sample = self.latest
    if sample is not None and 0. <= self.clock() - sample[0] <= self.MAX_SAMPLE_AGE:
      return sample[1]
    return None

  def run(self, end_event):
    while not end_event.is_set():
      self.sample()
      end_event.wait(self.SAMPLE_INTERVAL)
