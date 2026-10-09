import math
from pathlib import Path
import unittest

import csim
from csim_control import ControlConfig, ControlLoop

IMPLICIT = ('implicit_euler', 'implicit_midpoint', 'radau5', 'bdf1', 'bdf2')
LIE = ('lie_midpoint', 'lie_rk4')
HIGH_ORDER = ('dop853',)
ROOT = Path(__file__).resolve().parents[2] / 'examples/models/rigid_payload'


class AdvancedIntegratorTests(unittest.TestCase):
    def test_freefall_pendulum_and_exact_external_clock(self):
        for method in IMPLICIT + LIE + HIGH_ORDER:
            model = csim.DroneModel(timestep=.01, integrator=method, rtol=1e-5, atol=1e-8)
            data = csim.make_data(model, position_W=[0, 0, 5])
            for _ in range(10):
                csim.step(model, data)
            state = csim.get_state(model, data)
            self.assertAlmostEqual(state['time'], .1)
            self.assertAlmostEqual(state['velocity_W'][2], -.1*model.gravity, places=10)
            expected = 5 - .005*model.gravity
            if method == 'implicit_euler':
                expected -= model.gravity*.01*.1/2
            tolerance = .002 if method == 'bdf1' else 1e-9
            self.assertLess(abs(state['position_W'][2] - expected), tolerance)
        for method in IMPLICIT + HIGH_ORDER + ('rattle',):
            model = csim.PendulumModel(timestep=.002, integrator=method, rtol=1e-5)
            data = csim.make_data(model, angle=.2)
            initial = csim.get_state(model, data)['energy']
            for _ in range(50):
                csim.step(model, data)
            self.assertLess(abs(csim.get_state(model, data)['energy']-initial), .001)

    def test_point_and_rigid_cable_constraints(self):
        for method in IMPLICIT + LIE + HIGH_ORDER + ('rattle',):
            model = csim.SuspendedPayloadModel(timestep=.002, integrator=method, rtol=1e-5,
                                              cable_mode='hybrid' if method == 'rattle' else 'taut')
            data = csim.make_data(model, thrust=12, cable_direction_W=[.1, 0, -math.sqrt(.99)])
            for _ in range(10):
                csim.step(model, data)
            state = csim.get_state(model, data)
            self.assertAlmostEqual(sum(value*value for value in state['cable_direction_W']), 1, places=12)
            self.assertAlmostEqual(sum(a*b for a,b in zip(state['cable_direction_W'], state['cable_angular_velocity_W'])), 0, places=12)
        for method in IMPLICIT + LIE + HIGH_ORDER:
            model = csim.load_suspended_model(str(ROOT/'drone.urdf'), str(ROOT/'cable.json'), str(ROOT/'payload.urdf'),
                                              integrator=method, timestep=.002, rtol=1e-5, atol=1e-8)
            data = csim.make_data(model, thrust=15)
            for _ in range(3):
                csim.step(model, data)
            state = csim.get_state(model, data)
            self.assertAlmostEqual(state['cable_distance'], 1, places=11)
            self.assertAlmostEqual(state['cable_radial_velocity'], 0, places=11)
            self.assertAlmostEqual(sum(value*value for value in state['payload_q_WP']), 1, places=12)
            self.assertEqual(csim.get_config(model)['integrator'], method)

    def test_rattle_hybrid_impact_and_model_restrictions(self):
        model = csim.SuspendedPayloadModel(timestep=.02, integrator='rattle', cable_mode='hybrid')
        data = csim.make_data(model, thrust=0, cable_mode='slack', payload_position_W=[0, 0, -.99], payload_velocity_W=[0, 0, -1])
        csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertEqual(state['cable_events'][0]['type'], 'impact')
        self.assertAlmostEqual(state['cable_events'][0]['time'], .01, places=8)
        with self.assertRaises(ValueError):
            csim.DroneModel(integrator='rattle')
        with self.assertRaises(ValueError):
            csim.SuspendedPayloadModel(integrator='rattle', drone_drag=csim.DragConfig(k1=.1))
        with self.assertRaises(ValueError):
            csim.load_suspended_model(str(ROOT/'drone.urdf'), str(ROOT/'cable.json'), str(ROOT/'payload.urdf'), integrator='rattle')

    def test_implicit_budget_failure_and_batch_parity(self):
        for method in ('radau5', 'bdf1', 'bdf2'):
            model = csim.DroneModel(timestep=.1, integrator=method, max_substeps=1)
            data = csim.make_data(model, angular_velocity_B=[0, 0, 5])
            before = csim.get_state(model, data)
            with self.assertRaises(RuntimeError):
                csim.step(model, data)
            self.assertEqual(csim.get_state(model, data), before)
        model = csim.DroneModel(timestep=1, integrator='dop853', rtol=1e-12, atol=1e-14, max_substeps=1)
        data = csim.make_data(model, angular_velocity_B=[0, 0, 20])
        before = csim.get_state(model, data)
        with self.assertRaises(RuntimeError):
            csim.step(model, data)
        self.assertEqual(csim.get_state(model, data), before)
        for method in IMPLICIT + LIE + HIGH_ORDER:
            model = csim.DroneModel(integrator=method, rtol=1e-5)
            batch = csim.DroneBatch(model, 4, threads=2)
            single = csim.make_data(model)
            csim.set_control(model, single, model.gravity, [0, 0, .001])
            batch.step([[model.gravity, 0, 0, .001]]*4)
            csim.step(model, single)
            state = csim.get_state(model, single)
            expected = state['position_W'] + state['velocity_W'] + state['q_WB'] + state['angular_velocity_B']
            for row in batch.get_state()['state']:
                for actual, reference in zip(row, expected):
                    self.assertAlmostEqual(actual, reference, places=13)

    def test_ctbr_stays_in_python_and_supports_rigid_payload(self):
        for method in ('lie_rk4', 'radau5', 'dop853'):
            model = csim.load_suspended_model(str(ROOT/'drone.urdf'), str(ROOT/'cable.json'), str(ROOT/'payload.urdf'), integrator=method)
            data = csim.make_data(model, thrust=15)
            loop = ControlLoop(model, data, ControlConfig(mode='ctbr', controller_period=.002))
            loop.set_ctbr(15, [0, 0, .1])
            for _ in range(4):
                loop.step()
            self.assertEqual(loop.get_state()['actuation']['controller_updates'], 2)
            self.assertFalse(hasattr(model, 'set_ctbr'))
            self.assertFalse(hasattr(csim, 'set_ctbr'))


if __name__ == '__main__':
    unittest.main()
