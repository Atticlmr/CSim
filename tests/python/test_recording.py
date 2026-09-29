"""The Python experiment records poses without importing any graphics package."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("record_payload", Path(__file__).resolve().parents[2]/"examples/python/record_payload.py")
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class RecordingTests(unittest.TestCase):
    def test_poses_metadata_and_final_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"flight.csv"
            count = recorder.record(path, duration=0.022, sample_steps=5)
            with path.open(newline="") as file:
                reader = csv.DictReader(file)
                self.assertEqual(reader.fieldnames, recorder.HEADER)
                rows = [{k: float(v) for k, v in row.items()} for row in reader]
            self.assertEqual(count, 4)
            self.assertEqual(len(rows), 4)
            for row, time in zip(rows, (0, 0.01, 0.02, 0.022)):
                self.assertAlmostEqual(row["time"], time)
                length = math.sqrt(sum((row[f"drone_{a}"]-row[f"payload_{a}"])**2 for a in "xyz"))
                self.assertAlmostEqual(length, 1.2)
                self.assertGreater(row["tension"], 0)
                self.assertAlmostEqual(sum(row[f"q_{a}"]**2 for a in "wxyz"), 1)
            metadata = json.loads(Path(str(path)+".json").read_text())
            self.assertEqual(metadata["format"], "csim-viewer-csv-v1")
            self.assertTrue(metadata["completed"])
            self.assertEqual(metadata["samples"], count)
            self.assertAlmostEqual(metadata["final_time"], 0.022)

    def test_invalid_options_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"flight.csv"
            for args in ({"duration": 0}, {"duration": math.nan}, {"timestep": -1},
                         {"duration": 0.003}, {"sample_steps": 0}):
                with self.assertRaises(ValueError):
                    recorder.record(path, **args)
                self.assertFalse(path.exists())
            path.write_text("existing experiment\n")
            with self.assertRaises(FileExistsError):
                recorder.record(path, duration=0.02)
            self.assertEqual(path.read_text(), "existing experiment\n")


if __name__ == "__main__":
    unittest.main()
