import json
from pathlib import Path
import subprocess
import sys
import unittest
import tempfile

EXAMPLES=Path(__file__).resolve().parents[2]/'examples/python'
class FlightExamplesTests(unittest.TestCase):
    def run_example(self,name,*options):
        with tempfile.TemporaryDirectory() as directory:
            return json.loads(subprocess.check_output([sys.executable,str(EXAMPLES/name),'--headless','--output-dir',str(Path(directory)/'run'),*options],text=True))
    def test_hover_and_tracking(self):
        self.assertLess(self.run_example('hover.py')['position_rmse_last_quarter_m'],1e-6)
        self.assertLess(self.run_example('tracking.py')['position_rmse_last_quarter_m'],.03)
    def test_swing_feedback_against_same_position_controller(self):
        on=self.run_example('payload_swing.py')
        off=self.run_example('payload_swing.py','--no-swing-damping')
        self.assertLess(on['swing_rms_last_quarter_rad'],off['swing_rms_last_quarter_rad']*.1)
        self.assertGreater(on['state']['tension'],0)

if __name__=='__main__': unittest.main()
