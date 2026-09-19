import copy
import hashlib
from pathlib import Path
import tempfile
import unittest

from tools.bluepilot_chestnut.preflight import verify_model_files


class PreflightTest(unittest.TestCase):
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
