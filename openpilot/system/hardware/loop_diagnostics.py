"""Observe stalled hardware publication without manufacturing a fresh heartbeat.

The existing hardwared supervisor calls poll(); the hardware worker only records
successful publication. No worker interruption, file reads, local-variable dumps,
timeout changes, or recovery actions are performed here.
"""
import sys
import time


class HardwareLoopDiagnostics:
  STALL_SECONDS = 2.0
  MAX_STACK_FRAMES = 16

  def __init__(self, clock=time.monotonic):
    self.clock = clock
    self.last_publication = None
    self.reported_publication = None
    self.last_gap = None
    self.reported_gap = None

  def published(self):
    # Tuple replacement is atomic under CPython. Keep a generation so an equal
    # clock reading still denotes progress; the supervisor never writes it.
    previous = self.last_publication
    now = self.clock()
    if previous is not None and now - previous[1] > self.STALL_SECONDS:
      self.last_gap = (previous[1], now)
    self.last_publication = (0 if previous is None else previous[0] + 1, now)

  def poll(self, thread_id, frames=None):
    publication = self.last_publication
    if publication is None or thread_id is None:
      return None
    now = self.clock()
    if self.reported_publication is not None and publication != self.reported_publication:
      previous = self.reported_publication
      self.reported_publication = None
      gap = self.last_gap
      self.reported_gap = gap
      return {'event': 'hardwareStatusResumed',
              'publication_gap_s': gap[1] - gap[0] if gap is not None and gap[0] == previous[1] else None}
    if self.last_gap is not None and self.last_gap != self.reported_gap:
      # A C call holding the GIL (or supervisor scheduling delay) can prevent
      # sampling during the stall. Retain the measured gap without pretending
      # that a stack captured after recovery identifies the blocking operation.
      gap = self.last_gap
      self.reported_gap = gap
      return {'event': 'hardwareStatusGap', 'publication_gap_s': gap[1] - gap[0], 'stack_unavailable': True}
    if now - publication[1] <= self.STALL_SECONDS or publication == self.reported_publication:
      return None

    frame = (sys._current_frames() if frames is None else frames).get(thread_id)
    if frame is None:
      return None
    stack = []
    try:
      while frame is not None and len(stack) < self.MAX_STACK_FRAMES:
        # Deliberately do not retain frame objects or inspect locals/arguments.
        stack.append({'file': frame.f_code.co_filename, 'line': frame.f_lineno, 'function': frame.f_code.co_name})
        frame = frame.f_back
    finally:
      del frame
    # A worker may have resumed while its stack was sampled. Do not attribute
    # the subsequent code to the old publication delay.
    if self.last_publication != publication:
      return None
    self.reported_publication = publication
    return {'event': 'hardwareStatusStalled', 'since_publication_s': now - publication[1], 'stack': stack}
