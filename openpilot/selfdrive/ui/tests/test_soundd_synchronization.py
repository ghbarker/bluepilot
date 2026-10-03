import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from openpilot.selfdrive.ui import soundd
from openpilot.selfdrive.ui.bp import soundd_bp


class TestSounddSynchronization(unittest.TestCase):
  def daemon(self, cls=soundd.Soundd):
    # Exercise production playback methods without opening a device or loading assets.
    daemon = cls.__new__(cls)
    daemon._sound_lock = threading.RLock()
    daemon.current_alert = soundd.AudibleAlert.promptRepeat
    daemon.current_sound_frame = 0
    daemon.current_volume = 1.0
    daemon.pending_stop = False
    daemon.ramp_start_volume = 0.1
    daemon.ramp_start_time = 0.0
    daemon.loaded_sounds = {alert: np.ones(4, dtype=np.float32) for alert in soundd.sound_list}
    daemon.should_play_sound = lambda alert: alert != soundd.AudibleAlert.none
    daemon.params = Mock()
    return daemon

  def test_alert_update_cannot_change_alert_mid_callback_lookup(self):
    daemon = self.daemon()
    callback_entered = threading.Event()
    allow_callback = threading.Event()
    update_attempted = threading.Event()
    update_finished = threading.Event()
    failures = []
    output = []

    def should_play(alert):
      callback_entered.set()
      if not allow_callback.wait(2.0):
        raise TimeoutError('test callback was not released')
      return alert != soundd.AudibleAlert.none

    def callback():
      try:
        output.append(daemon.get_sound_data(4))
      except Exception as exc:
        failures.append(exc)

    def update():
      update_attempted.set()
      try:
        daemon.update_alert(soundd.AudibleAlert.none)
      except Exception as exc:
        failures.append(exc)
      finally:
        update_finished.set()

    daemon.should_play_sound = should_play
    playback = threading.Thread(target=callback)
    updater = threading.Thread(target=update)
    playback.start()
    try:
      self.assertTrue(callback_entered.wait(2.0))
      updater.start()
      self.assertTrue(update_attempted.wait(2.0))
      # Without serialization, update_alert switches to none before the callback
      # indexes sound_list, terminating the callback with KeyError.
      self.assertFalse(update_finished.wait(0.05))
    finally:
      allow_callback.set()
      playback.join(2.0)
      if updater.ident is not None:
        updater.join(2.0)
    self.assertFalse(playback.is_alive())
    self.assertFalse(updater.is_alive())
    self.assertEqual(failures, [])
    np.testing.assert_array_equal(output[0], np.ones(4, dtype=np.float32))
    self.assertTrue(daemon.pending_stop)
    daemon.get_sound_data(4)
    self.assertEqual(daemon.current_alert, soundd.AudibleAlert.none)

  def test_immediate_warning_preempts_pending_loop_stop(self):
    daemon = self.daemon()
    daemon.current_sound_frame = 5
    daemon.update_alert(soundd.AudibleAlert.none)
    self.assertTrue(daemon.pending_stop)
    daemon.update_alert(soundd.AudibleAlert.warningImmediate)
    self.assertFalse(daemon.pending_stop)
    self.assertEqual(daemon.current_sound_frame, 0)
    self.assertEqual(daemon.current_alert, soundd.AudibleAlert.warningImmediate)
    np.testing.assert_array_equal(daemon.get_sound_data(4), np.ones(4, dtype=np.float32))

  def test_custom_bank_is_published_only_after_all_assets_are_ready(self):
    daemon = self.daemon(soundd_bp.SounddBP)
    old_bank = daemon.loaded_sounds
    stock_bank = {alert: np.full(4, 2., dtype=np.float32) for alert in soundd.sound_list}
    custom_sound = np.full(6, 3., dtype=np.float32)
    loading_custom = threading.Event()
    allow_load = threading.Event()
    failures = []

    def load_custom(path):
      loading_custom.set()
      if not allow_load.wait(2.0):
        raise TimeoutError('test asset load was not released')
      return custom_sound

    def reload_bank():
      try:
        daemon.load_sounds()
      except Exception as exc:
        failures.append(exc)

    selection = soundd_bp.CustomSoundSelection.TESLA
    with patch.object(soundd.Soundd, '_load_sound_bank', return_value=(stock_bank, 'theme')), \
         patch.object(soundd_bp, '_requested_sound_selection', return_value=selection), \
         patch.dict(soundd_bp.SOUND_PACK_FILES, {selection: {soundd.AudibleAlert.engage: 'custom.wav'}}), \
         patch.object(soundd_bp, '_load_mono_sound', side_effect=load_custom):
      loader = threading.Thread(target=reload_bank)
      loader.start()
      try:
        self.assertTrue(loading_custom.wait(2.0))
        self.assertIs(daemon.loaded_sounds, old_bank)
        # Loading must not hold the callback lock or replace a partial bank.
        np.testing.assert_array_equal(daemon.get_sound_data(4), np.ones(4, dtype=np.float32))
      finally:
        allow_load.set()
        loader.join(2.0)
    self.assertFalse(loader.is_alive())
    self.assertEqual(failures, [])
    self.assertEqual(daemon._theme_pack_name, 'theme')
    self.assertEqual(set(daemon.loaded_sounds), set(soundd.sound_list))
    np.testing.assert_array_equal(daemon.loaded_sounds[soundd.AudibleAlert.engage], custom_sound)
    np.testing.assert_array_equal(daemon.loaded_sounds[soundd.AudibleAlert.warningImmediate], stock_bank[soundd.AudibleAlert.warningImmediate])


if __name__ == '__main__':
  unittest.main()
