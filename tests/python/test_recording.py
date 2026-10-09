"""The Python experiment records poses without importing any graphics package."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import csim

spec = importlib.util.spec_from_file_location("record_payload", Path(__file__).resolve().parents[2]/"examples/python/record_payload.py")
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class RecordingTests(unittest.TestCase):
    def test_rigid_csv_contains_orientation_offset_attachments_and_slack(self):
        root=Path(__file__).resolve().parents[2]/'examples/models/rigid_payload'
        model=csim.load_suspended_model(str(root/'drone.json'),str(root/'cable.json'),str(root/'payload.json'),timestep=.002)
        initial={'mode':'slack','payload_position_W':[0,0,-1.19],'payload_velocity_W':[0,0,-1]}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'rigid.csv'
            recorder.record(path,duration=.02,sample_steps=1,model=model,initial=initial,thrust=0.)
            with path.open(newline='') as stream:
                reader=csv.DictReader(stream)
                self.assertEqual(reader.fieldnames,recorder.RIGID_HEADER)
                rows=[{key:float(value) for key,value in row.items()} for row in reader]
            self.assertEqual(rows[0]['cable_slack'],1.)
            self.assertEqual(rows[0]['tension'],0.)
            self.assertTrue(any(abs(row['payload_q_y'])>1e-5 for row in rows))
            for row in rows:
                distance=math.sqrt(sum((row[f'payload_attachment_{axis}']-row[f'drone_attachment_{axis}'])**2 for axis in 'xyz'))
                self.assertLessEqual(distance,row['cable_length']+1e-9)
            metadata=json.loads(Path(str(path)+'.json').read_text())
            self.assertEqual(metadata['format'],'csim-viewer-csv-v2')
            self.assertEqual(metadata['model'],csim.get_config(model))

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
