import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from openpilot.selfdrive.ui.bp.lib import theme_pack, theme_scene
from openpilot.selfdrive.ui.bp.lib.theme_recovery import ThemeRecovery


class TestThemeRecovery(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.marker = Path(self.tmp.name) / 'activation'
    self.params = Mock()
    self.params.get.return_value = 'halloween_week'
    self.params.get_bool.return_value = False
    self.log = Mock()

  def test_interrupted_activation_disables_manual_and_automatic_selection(self):
    guard = ThemeRecovery(self.params, self.marker, self.log)
    self.assertTrue(guard.begin('halloween_week'))
    self.assertTrue(self.marker.exists())
    restarted = ThemeRecovery(self.params, self.marker, self.log)
    self.assertTrue(restarted.disabled)
    self.params.put.assert_called_once_with('BPThemePack', '', block=True)
    self.params.put_bool.assert_called_once_with('BPThemeAutoSeasonal', False, block=True)
    self.assertFalse(self.marker.exists())
    self.assertFalse(restarted.begin('rad_racer'))
    # Recovery is idempotent on another startup, and preserves all pack files.
    again = ThemeRecovery(self.params, self.marker, self.log)
    self.assertFalse(again.disabled)
    self.assertEqual(self.params.put.call_count, 1)

  def test_completed_load_and_first_onroad_render_each_have_one_marker(self):
    guard = ThemeRecovery(self.params, self.marker, self.log)
    with patch('openpilot.selfdrive.ui.bp.lib.theme_recovery.os.fsync') as fsync:
      for _ in range(59):
        self.assertTrue(guard.begin('rad_racer'))
        guard.frame_rendered()
      self.assertTrue(self.marker.exists())
      guard.frame_rendered()
      self.assertFalse(self.marker.exists())
      self.assertEqual(fsync.call_count, 1)
      for _ in range(60):
        self.assertTrue(guard.begin('rad_racer'))
        self.assertTrue(guard.begin_render('rad_racer'))
        guard.frame_rendered()
      self.assertFalse(self.marker.exists())
      self.assertEqual(fsync.call_count, 2)
      guard.begin_render('halloween_week')
      self.assertTrue(self.marker.exists())
      guard.close()
      self.assertFalse(self.marker.exists())

  def test_persist_failure_keeps_in_memory_fallback_and_marker(self):
    self.marker.write_text('rad_racer')
    self.params.put.side_effect = OSError('read-only')
    guard = ThemeRecovery(self.params, self.marker, self.log)
    self.assertTrue(guard.disabled)
    self.assertTrue(self.marker.exists())
    self.assertFalse(guard.begin('rad_racer'))
    guard.close()
    self.assertTrue(self.marker.exists())
    restarted = ThemeRecovery(self.params, self.marker, self.log)
    self.assertTrue(restarted.disabled)

  def test_marker_failure_disables_theme_without_raising(self):
    parent = Path(self.tmp.name) / 'not_a_directory'
    parent.write_text('unrelated')
    guard = ThemeRecovery(self.params, parent / 'activation', self.log)
    self.assertFalse(guard.begin('rad_racer'))
    self.assertTrue(guard.disabled)
    self.assertEqual(parent.read_text(), 'unrelated')

  def test_marker_is_written_before_pack_loading(self):
    guard = ThemeRecovery(self.params, self.marker, self.log)
    def resolve(name):
      self.assertTrue(self.marker.exists())
      self.assertEqual(self.marker.read_text(), name)
      return object()
    with patch.object(theme_pack, '_ui_recovery', guard), patch.object(theme_pack, 'Params', return_value=self.params), \
         patch.object(theme_pack, '_cache', {'checked_at': 0., 'name': None, 'pack': None}), \
         patch.object(theme_pack, '_resolve', side_effect=resolve):
      self.assertIsNotNone(theme_pack.get_active_pack(force=True))

  def test_initial_auto_seasonal_scan_is_guarded_and_clean_exit_keeps_selection(self):
    self.params.get.return_value = ''
    self.params.get_bool.return_value = True
    self.params.get_param_path.return_value = str(Path(self.tmp.name) / 'd')
    marker = Path(self.tmp.name) / 'd_bp_theme_activation'
    def seasonal():
      self.assertTrue(marker.exists())
      return 'halloween_week'
    with patch.object(theme_pack, '_ui_recovery', None), patch.object(theme_pack, 'Params', return_value=self.params), \
         patch.object(theme_pack, '_cache', {}), patch.object(theme_pack, 'seasonal_pack', side_effect=seasonal), \
         patch.object(theme_pack, '_resolve', return_value=object()):
      theme_pack.initialize_ui_recovery()
      self.assertEqual(theme_pack._effective_name(self.params), 'halloween_week')
      theme_pack._ui_recovery.close()
    self.assertFalse(marker.exists())
    self.params.put.assert_not_called()
    self.params.put_bool.assert_not_called()

  def test_disabled_guard_bypasses_cached_pack_and_scene_immediately(self):
    guard = ThemeRecovery(self.params, self.marker, self.log)
    guard.disabled = True
    with patch.object(theme_pack, '_ui_recovery', guard), \
         patch.object(theme_pack, '_cache', {'checked_at': float('inf'), 'name': 'bad', 'pack': object()}), \
         patch.object(theme_scene, '_scene_cache', {'checked_at': float('inf'), 'key': 'bad', 'scene': object()}):
      self.assertIsNone(theme_pack.get_active_pack())
      self.assertIsNone(theme_scene.active_scene())
      self.assertEqual(theme_pack._effective_name(self.params), '')

  def test_malformed_color_and_season_files_fall_back(self):
    root = Path(self.tmp.name)
    (root / 'colors').mkdir()
    for payload in ([], {'Accent': {'red': -1, 'green': 500, 'blue': 2, 'alpha': 255}},
                    {'Accent': {'red': float('inf'), 'green': 2, 'blue': 2, 'alpha': 255}}):
      with self.subTest(colors=payload):
        (root / 'colors/colors.json').write_text(json.dumps(payload))
        self.assertEqual(theme_pack.ThemePack('bad', str(root)).colors, {})
    for payload in ([], {'start': [], 'end': '12-31'}, {'start': '12-01', 'end': 31},
                    {'anchor': 'easter', 'offset_start': float('inf')}):
      with self.subTest(season=payload):
        (root / 'season.json').write_text(json.dumps(payload))
        self.assertIsNone(theme_pack._season_window(str(root), 2026))

  def test_invalid_scene_containers_return_colors_only_default(self):
    path = Path(self.tmp.name) / 'scene.json'
    for payload in ([], 1, {'version': 1, 'sky': [1]}, {'version': 1, 'foreground': [1]},
                    {'version': 1, 'layers': 3}, {'version': float('inf')}):
      with self.subTest(scene=payload):
        path.write_text(json.dumps(payload))
        self.assertIsNone(theme_scene.load_scene_spec(path))
