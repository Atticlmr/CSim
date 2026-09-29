import gc
import math
import unittest

import csim


class PendulumSimulationTests(unittest.TestCase):
    def test_physical_step_and_snapshot(self):
        model = csim.PendulumModel(gravity=9.81, timestep=0.01, pivot_W=[1, 2, 3])
        data = csim.make_data(model, angle=0.2)
        before = csim.get_state(model, data)
        self.assertIsNone(csim.step(model, data))
        state = csim.get_state(model, data)
        self.assertEqual(state['time'], 0.01)
        a, theta, h = 9.81, 0.2, 0.01
        expected_angle = theta - a * math.sin(theta) * h**2 / 2 + a*a*math.sin(theta)*math.cos(theta)*h**4/24
        expected_rate = -a*math.sin(theta)*h + a*a*math.sin(theta)*math.cos(theta)*h**3/6
        self.assertAlmostEqual(state['angle'], expected_angle, delta=1e-11)
        self.assertAlmostEqual(state['angular_velocity'], expected_rate, delta=1e-9)
        self.assertAlmostEqual(math.dist(state['position_W'], model.pivot_W), model.length, places=14)
        self.assertAlmostEqual(sum(a*b for a, b in zip(state['cable_direction_W'], state['velocity_W'])), 0, places=14)
        self.assertGreater(state['tension'], 0)
        self.assertEqual(state['mode'], 'taut')
        self.assertEqual(before['time'], 0)
        self.assertEqual(before['angle'], 0.2)
        before['position_W'][0] = 999
        state['angle'] = 999
        self.assertLess(csim.get_state(model, data)['angle'], 0.2)
        self.assertNotEqual(csim.get_state(model, data)['position_W'][0], 999)

    def test_equilibrium_reset_and_independent_data(self):
        model = csim.PendulumModel(length=2, mass=3, gravity=9.81)
        rest = csim.make_data(model)
        swing = csim.make_data(model, angle=0.4)
        for _ in range(100):
            csim.step(model, rest)
        state = csim.get_state(model, rest)
        self.assertEqual(state['angle'], 0)
        self.assertEqual(state['angular_velocity'], 0)
        self.assertEqual(state['position_W'], [0, 0, -2])
        self.assertAlmostEqual(state['tension'], 3*9.81)
        self.assertEqual(state['energy'], 0)
        self.assertEqual(csim.get_state(model, swing)['time'], 0)
        csim.reset(model, rest, angle=-0.1, angular_velocity=0.2)
        state = csim.get_state(model, rest)
        self.assertEqual(state['time'], 0)
        self.assertEqual(state['angle'], -0.1)
        self.assertEqual(state['angular_velocity'], 0.2)

    def test_ownership_and_model_identity(self):
        model = csim.PendulumModel()
        data = csim.make_data(model, angle=0.3)
        with self.assertRaises(AttributeError):
            model.timestep = 0.2
        pivot_copy = model.pivot_W
        pivot_copy[0] = 99
        self.assertEqual(model.pivot_W, [0, 0, 0])
        other = csim.PendulumModel()
        for call in (csim.step, csim.get_state, csim.reset):
            with self.assertRaises(ValueError):
                call(other, data)
        del model
        gc.collect()
        csim.step(data.model, data)
        snapshot = csim.get_state(data.model, data)
        del data
        gc.collect()
        self.assertEqual(snapshot['time'], 0.001)
        self.assertTrue(all(math.isfinite(x) for x in snapshot['position_W']))

    def test_energy_over_time(self):
        model = csim.PendulumModel(timestep=0.01)
        data = csim.make_data(model, angle=0.7)
        energy = csim.get_state(model, data)['energy']
        for _ in range(2000):
            csim.step(model, data)
        final = csim.get_state(model, data)
        self.assertAlmostEqual(final['time'], 20, delta=1e-11)
        self.assertLess(abs(final['energy']/energy - 1), 1e-7)

    def test_invalid_input_and_atomic_failure(self):
        for bad in (0, -1, math.inf, math.nan):
            for field in ('length', 'mass', 'gravity', 'timestep'):
                with self.assertRaises(ValueError):
                    csim.PendulumModel(**{field: bad})
        with self.assertRaises(TypeError):
            csim.PendulumModel(pivot_W=[0, 0])
        model = csim.PendulumModel(timestep=2)
        data = csim.make_data(model, angle=0.7)
        before = csim.get_state(model, data)
        with self.assertRaises(csim.PendulumDomainError):
            csim.step(model, data)
        self.assertEqual(csim.get_state(model, data), before)
        with self.assertRaises(csim.PendulumDomainError):
            csim.reset(model, data, angle=2)
        self.assertEqual(csim.get_state(model, data), before)
        for angle, rate in ((2, 0), (0, 10)):
            with self.assertRaises(csim.PendulumDomainError):
                csim.make_data(model, angle=angle, angular_velocity=rate)
        with self.assertRaises(ValueError):
            csim.make_data(model, angle=math.nan)
        tiny = csim.PendulumModel(timestep=math.ulp(0.0))
        tiny_data = csim.make_data(tiny)
        with self.assertRaises(OverflowError):
            csim.step(tiny, tiny_data)
        self.assertEqual(csim.get_state(tiny, tiny_data)['time'], 0)


if __name__ == '__main__':
    unittest.main()
