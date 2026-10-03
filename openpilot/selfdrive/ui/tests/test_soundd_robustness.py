from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
import wave

import numpy as np

from openpilot.selfdrive.ui import soundd


class TestSounddRobustness(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.root = Path(self.tmp.name)

  def wav(self, name, frames=b'\x01\x00' * 8, channels=1, rate=soundd.SAMPLE_RATE):
    path = self.root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), 'wb') as f:
      f.setnchannels(channels)
      f.setsampwidth(2)
      f.setframerate(rate)
      f.writeframes(frames)
    return str(path)

  def daemon(self):
    daemon = soundd.Soundd.__new__(soundd.Soundd)
    daemon._sound_lock = threading.RLock()
    return daemon

  def test_empty_and_truncated_theme_assets_are_rejected_before_playback(self):
    valid = self.wav('valid.wav')
    np.testing.assert_array_equal(soundd.load_sound(valid), np.full(8, 1 / 32768, dtype=np.float32))
    truncated = self.wav('truncated.wav')
    Path(truncated).write_bytes(Path(truncated).read_bytes()[:-2])
    for path in (self.wav('empty.wav', b''), truncated, self.wav('stereo.wav', channels=2),
                 self.wav('wrong_rate.wav', rate=44100)):
      with self.subTest(path=path), self.assertRaises(ValueError):
        soundd.load_sound(path)
    short_header = self.root / 'header.wav'
    short_header.write_bytes(b'RIFF')
    with self.assertRaises((EOFError, wave.Error)):
      soundd.load_sound(str(short_header))

  def test_invalid_theme_sound_uses_stock(self):
    stock = self.wav('openpilot/selfdrive/assets/sounds/warning.wav')
    empty = self.wav('empty.wav', b'')
    daemon = self.daemon()
    pack = Mock(name='pack')
    pack.name = 'bad_pack'
    pack.sound_path.return_value = empty
    with patch.object(soundd, 'BASEDIR', str(self.root)), \
         patch.object(soundd, 'sound_list', {7: ('warning.wav', None, 1.)}), \
         patch.object(soundd.theme_pack, 'get_active_pack', return_value=pack):
      daemon.load_sounds()
    np.testing.assert_array_equal(daemon.loaded_sounds[7], soundd.load_sound(stock))

  def test_failed_reload_keeps_previous_complete_bank(self):
    daemon = self.daemon()
    bank = {7: np.ones(8, dtype=np.float32)}
    daemon.loaded_sounds = bank
    daemon._theme_pack_name = 'previous'
    with patch.object(soundd, 'BASEDIR', str(self.root)), \
         patch.object(soundd, 'sound_list', {7: ('missing.wav', None, 1.)}), \
         patch.object(soundd.theme_pack, 'get_active_pack', return_value=None), self.assertRaises(OSError):
      daemon.load_sounds()
    self.assertIs(daemon.loaded_sounds, bank)
    self.assertEqual(daemon._theme_pack_name, 'previous')

  def test_start_failure_closes_failed_stream_and_retries_start(self):
    daemon = self.daemon()
    failed, good = Mock(), Mock()
    failed.start.side_effect = OSError('output not ready')
    with patch.object(daemon, 'get_stream', side_effect=[failed, good]) as get_stream, \
         patch('openpilot.common.utils.time.sleep') as sleep:
      with daemon.audio_stream(object()) as stream:
        self.assertIs(stream, good)
        self.assertEqual(get_stream.call_count, 2)
        failed.close.assert_called_once()
        good.start.assert_called_once()
        good.close.assert_not_called()
      good.stop.assert_called_once()
      good.close.assert_called_once()
      sleep.assert_called_once_with(3)

  def test_start_retries_remain_bounded_and_failure_is_visible(self):
    daemon = self.daemon()
    failed = Mock()
    failed.start.side_effect = OSError('output unavailable')
    with patch.object(daemon, 'get_stream', return_value=failed), \
         patch('openpilot.common.utils.time.sleep'), self.assertRaises(Exception):
      daemon.start_stream(object())
    self.assertEqual(failed.start.call_count, 10)
    self.assertEqual(failed.close.call_count, 10)
