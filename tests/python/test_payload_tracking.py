"""Check trajectory derivatives, cable geometry and closed-loop tracking."""

import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


EXAMPLE = Path(__file__).resolve().parents[2] / 'examples/python/payload_tracking.py'
SPEC = importlib.util.spec_from_file_location('payload_tracking_example', EXAMPLE)
tracking = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tracking)


class PayloadReferenceTests(unittest.TestCase):
    def test_analytic_derivatives_during_and_after_startup(self):
        h = 1e-5
        for t in (0.3, 1.7, 3.8, 5.3, 11.0):
            before = tracking.payload_reference(t - h)
            current = tracking.payload_reference(t)
            after = tracking.payload_reference(t + h)
            for order in range(4):
                for axis in range(3):
                    with self.subTest(t=t, order=order, axis=axis):
                        estimate = (after[order][axis] - before[order][axis]) / (2*h)
                        self.assertAlmostEqual(estimate, current[order + 1][axis], delta=1e-4)

    def test_startup_continuity_and_requested_peak_speed(self):
        initial = tracking.payload_reference(0.0)
        self.assertEqual(initial[0], [0.0, 0.0, 4.0])
        for derivative in initial[1:]:
            self.assertEqual(derivative, [0.0, 0.0, 0.0])
        for before, after in zip(tracking.payload_reference(4.0 - 1e-8),
                                 tracking.payload_reference(4.0 + 1e-8)):
            for a, b in zip(before, after):
                self.assertAlmostEqual(a, b, delta=1e-5)
        for speed in (0.5, 1.0):
            # At cos(phase)^2 = 3/8, sin(phase)^2 + sin(2*phase)^2 is maximal.
            phase = 2*math.pi + math.acos(math.sqrt(3/8))
            t = 2.0 + phase / (speed / 0.625)
            velocity = tracking.payload_reference(t, speed)[1]
            self.assertAlmostEqual(math.sqrt(sum(x*x for x in velocity)), speed, places=12)

    def test_cable_derivatives_and_unit_sphere_constraints(self):
        h = 1e-5

        def cable(t):
            return tracking.cable_reference(*tracking.payload_reference(t)[2:], 9.80665)

        for t in (0.5, 2.2, 3.9, 7.0):
            direction, rate, acceleration = cable(t)
            self.assertAlmostEqual(tracking.dot(direction, direction), 1.0, places=12)
            self.assertAlmostEqual(tracking.dot(direction, rate), 0.0, places=12)
            self.assertAlmostEqual(tracking.dot(direction, acceleration),
                                   -tracking.dot(rate, rate), places=12)
            before, after = cable(t - h), cable(t + h)
            for order, derivative in enumerate((rate, acceleration)):
                for axis in range(3):
                    estimate = (after[order][axis] - before[order][axis]) / (2*h)
                    self.assertAlmostEqual(estimate, derivative[axis], delta=1e-5)


class PayloadTrackingTests(unittest.TestCase):
    def run_example(self, *options):
        result = subprocess.run(
            [sys.executable, str(EXAMPLE), '--headless', *options],
            capture_output=True, text=True, timeout=30,
        )
        self.assertTrue(result.stdout.strip(), result.stderr)
        return result.returncode, json.loads(result.stdout)

    def test_fast_tracking_and_csv_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'tracking.csv'
            code, result = self.run_example('--speed', '1.0', '--csv', str(path))
            self.assertEqual(code, 0, result)
            self.assertEqual(result['status'], 'completed')
            self.assertEqual(result['accepted_steps'], 10000)
            self.assertLess(result['payload_position_rmse_m'], 0.03)
            self.assertLess(result['payload_position_max_error_m'], 0.06)
            self.assertAlmostEqual(result['peak_reference_speed_m_s'], 1.0, delta=1e-4)
            self.assertGreater(result['peak_payload_speed_m_s'], 0.95)
            self.assertLess(result['peak_payload_speed_m_s'], 1.15)
            self.assertGreater(result['minimum_tension_N'], 1.8)
            self.assertEqual(result['actuator_saturation_fraction'], 0.0)
            self.assertEqual(result['controller_limited_fraction'], 0.0)
            with path.open(newline='') as stream:
                rows = [{key: float(value) for key, value in row.items()}
                        for row in csv.DictReader(stream)]
            self.assertEqual(len(rows), 10001)
            self.assertTrue(all(math.isfinite(value) for row in rows for value in row.values()))
            self.assertAlmostEqual(rows[-1]['time_s'], result['duration_s'], places=8)
            # Recompute the time-weighted RMSE directly from logged positions.
            errors = [sum((row[f'payload_{axis}_m'] - row[f'reference_{axis}_m'])**2
                          for axis in 'xyz') for row in rows]
            integral = sum((errors[i] + errors[i-1]) / 2
                           * (rows[i]['time_s'] - rows[i-1]['time_s'])
                           for i in range(1, len(rows)))
            self.assertAlmostEqual(math.sqrt(integral / rows[-1]['time_s']),
                                   result['payload_position_rmse_m'], places=10)

    def test_slower_speed_still_tracks(self):
        code, result = self.run_example('--speed', '0.5')
        self.assertEqual(code, 0, result)
        self.assertLess(result['payload_position_rmse_m'], 0.01)
        self.assertAlmostEqual(result['peak_reference_speed_m_s'], 0.5, delta=1e-4)
        self.assertGreater(result['minimum_tension_N'], 0.0)

    def test_infeasible_speed_reports_last_accepted_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'failed.csv'
            code, result = self.run_example('--speed', '1.5', '--duration', '8', '--csv', str(path))
            self.assertEqual(code, 1, result)
            self.assertEqual(result['status'], 'failed')
            self.assertIn('tension', result['error'])
            self.assertGreater(result['accepted_steps'], 0)
            self.assertLess(result['duration_s'], 8.0)
            self.assertGreater(result['minimum_tension_N'], 0.0)
            with path.open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), result['accepted_steps'] + 1)
            self.assertAlmostEqual(float(rows[-1]['time_s']), result['duration_s'], places=8)
            for axis, value in zip('xyz', result['payload_position_W']):
                self.assertEqual(float(rows[-1][f'payload_{axis}_m']), value)


if __name__ == '__main__':
    unittest.main()
