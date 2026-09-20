import copy
import hashlib
from pathlib import Path
import tempfile
import unittest

from tools.bluepilot_chestnut.preflight import angle_fields, missing_legacy_fields, verify_model_files


class PreflightTest(unittest.TestCase):
  def test_struct_parser_does_not_read_adjacent_struct(self):
    header = "typedef struct { int max_angle_error; } Other;\ntypedef struct { int max_angle; } AngleSteeringLimits;"
    self.assertEqual(angle_fields(header), {"max_angle"})
    self.assertIn("max_angle_error", missing_legacy_fields(header))

  def test_unknown_struct_format_fails_closed(self):
    with self.assertRaises(ValueError):
      angle_fields("typedef int AngleSteeringLimits;")

  def test_missing_extra_corrupt_and_misnumbered_model_chunks_are_rejected(self):
    with tempfile.TemporaryDirectory() as scratch:
      root = Path(scratch)
      files = {"big_driving_tinygrad.pkl.chunkmanifest": b"18"}
      files.update({f"big_driving_tinygrad.pkl.chunk{i:02d}of18": bytes([i]) for i in range(1, 19)})
      manifest = {"directory": ".", "files": {}}
      for name, data in files.items():
        (root / name).write_bytes(data)
        manifest["files"][name] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
      verify_model_files(root, manifest)
      name = "big_driving_tinygrad.pkl.chunk01of18"
      (root / name).unlink()
      with self.assertRaises(ValueError):
        verify_model_files(root, manifest)
      (root / name).write_bytes(b"x")  # same length, wrong hash
      with self.assertRaises(ValueError):
        verify_model_files(root, manifest)
      (root / name).write_bytes(files[name])
      extra = root / "big_driving_tinygrad.pkl.chunk19of19"
      extra.write_bytes(b"x")
      with self.assertRaises(ValueError):
        verify_model_files(root, manifest)
      extra.unlink()
      invalid = copy.deepcopy(manifest)
      renamed = name.replace("01of18", "01of17")
      (root / name).rename(root / renamed)
      invalid["files"][renamed] = invalid["files"].pop(name)
      with self.assertRaises(ValueError):
        verify_model_files(root, invalid)


if __name__ == "__main__":
  unittest.main()
