"""UI-only recovery from an interrupted first render of a selected theme."""
import os
from pathlib import Path


class ThemeRecovery:
  def __init__(self, params, marker_path, log):
    self.params = params
    self.marker = Path(marker_path)
    self.log = log
    self.name = None
    self.rendered_name = None
    self.pending = False
    self.frames = 0
    self.disabled = False
    if self.marker.exists():
      self._disable("previous theme activation did not finish")

  def _clear_marker(self):
    try:
      self.marker.unlink(missing_ok=True)
    except OSError:
      self.log.exception("BluePilot: could not clear theme activation marker")

  def _disable(self, reason):
    # Keep plain UI usable even if persistent settings cannot be written.
    self.disabled = True
    self.pending = False
    self.log.warning(f"BluePilot: disabling themes: {reason}")
    try:
      self.params.put("BPThemePack", "", block=True)
      self.params.put_bool("BPThemeAutoSeasonal", False, block=True)
    except Exception:
      self.log.exception("BluePilot: could not persist theme recovery")
    else:
      self._clear_marker()

  def begin(self, name):
    if self.disabled:
      return False
    if name == self.name:
      return True
    self.name = name
    self.rendered_name = None
    self.frames = 0
    self.pending = False
    if not name:
      self._clear_marker()
      return True
    return self._arm(name)

  def _arm(self, name):
    self.frames = 0
    try:
      # One durable write per activation, never one write per frame. Preserve
      # the marker on exceptions or native crashes until the next UI startup.
      self.marker.parent.mkdir(parents=True, exist_ok=True)
      with self.marker.open('w') as marker:
        marker.write(name)
        marker.flush()
        os.fsync(marker.fileno())
    except OSError:
      self._disable("theme activation guard could not be saved")
      return False
    self.pending = True
    return True

  def begin_render(self, name):
    if not self.begin(name):
      return False
    if name and self.rendered_name != name:
      self.rendered_name = name
      # Offroad settings can finish loading before any onroad texture upload.
      # Guard that first actual render too, without writing on later frames.
      if not self.pending:
        return self._arm(name)
    return True

  def frame_rendered(self):
    if self.pending:
      self.frames += 1
      # Let deferred texture uploads and subsequent frames complete too.
      if self.frames >= 60:
        self._clear_marker()
        self.pending = False

  def close(self):
    # A normal UI shutdown is not a failed activation. Exception exits do not
    # call this method, so a persisted broken selection cannot loop forever.
    if not self.disabled:
      self._clear_marker()
