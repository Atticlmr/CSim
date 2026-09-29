"""Physical API tests: no mathematical classes or numerical callbacks required."""
import gc
import math
import unittest

import csim


class DroneTests(unittest.TestCase):
    def assertVectorNear(self, actual, expected, tolerance=1e-11):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertTrue(math.isfinite(a))
            self.assertLessEqual(abs(a - b), tolerance)

    def test_free_fall_and_hover(self):
        model = csim.DroneModel(mass=2, gravity=9.81, timestep=0.01)
        data = csim.make_data(model, position_W=[1, 2, 10], velocity_W=[0.4, -0.3, 0.5])
        for _ in range(100):
            self.assertIsNone(csim.step(model, data))
        state = csim.get_state(model, data)
        self.assertAlmostEqual(state["time"], 1)
        self.assertVectorNear(state["position_W"], [1.4, 1.7, 5.595])
        self.assertVectorNear(state["velocity_W"], [0.4, -0.3, -9.31])
        self.assertVectorNear(state["acceleration_W"], [0, 0, -9.81])
        self.assertVectorNear(state["q_WB"], [1, 0, 0, 0])
        self.assertIsNone(csim.reset(model, data, position_W=[0, 0, 5]))
        self.assertIsNone(csim.set_control(model, data, thrust=model.mass * model.gravity))
        for _ in range(1000):
            csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertVectorNear(state["position_W"], [0, 0, 5], 0)
        self.assertVectorNear(state["velocity_W"], [0, 0, 0], 0)
        self.assertEqual(state["control"], {"thrust": 19.62, "torque_B": [0, 0, 0]})

    def test_tilt_and_body_torque(self):
        model = csim.DroneModel(gravity=9.81, timestep=0.005)
        pitch = math.pi / 6
        data = csim.make_data(model, q_WB=[math.cos(pitch/2), 0, math.sin(pitch/2), 0])
        csim.set_control(model, data, thrust=9.81/math.cos(pitch))
        for _ in range(200):
            csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertVectorNear(state["position_W"], [0.5*9.81*math.tan(pitch), 0, 0])
        csim.reset(model, data, position_W=[0, 0, 5], angular_velocity_B=[0, 0, 0.2])
        csim.set_control(model, data, thrust=9.81, torque_B=[0, 0, 0.024])
        for _ in range(200):
            csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertVectorNear(state["angular_velocity_B"], [0, 0, 0.8])
        self.assertVectorNear(state["q_WB"], [math.cos(0.25), 0, 0, math.sin(0.25)])
        self.assertVectorNear(state["position_W"], [0, 0, 5])
        self.assertAlmostEqual(sum(v*v for v in state["q_WB"]), 1, places=14)

    def test_full_inertia_and_current_observables(self):
        inertia = [[2, 0.2, 0.1], [0.2, 3, 0.3], [0.1, 0.3, 4]]
        model = csim.DroneModel(inertia_B=inertia)
        inertia[0][0] = 99
        self.assertEqual(model.inertia_B[0][0], 2)
        data = csim.make_data(model, angular_velocity_B=[1, 2, 3])
        old = csim.get_state(model, data)
        csim.set_control(model, data, thrust=1, torque_B=[5.125, -5.025, 4.675])
        state = csim.get_state(model, data)
        self.assertEqual(state["time"], 0)
        self.assertVectorNear(state["angular_acceleration_B"], [0.5, -0.25, 0.75])
        self.assertVectorNear(state["angular_momentum_W"], [2.7, 7.1, 12.7])
        self.assertEqual(old["control"]["thrust"], 0)
        self.assertAlmostEqual(state["energy"], 27.5)

    def test_copies_reset_lifetime_and_model_identity(self):
        model = csim.DroneModel()
        data = csim.make_data(model, q_WB=[2, 0, 0, 0])
        independent = csim.make_data(model)
        csim.set_control(model, data, thrust=12, torque_B=[0, 0, 0.01])
        state = csim.get_state(model, data)
        state["position_W"][0] = 99
        state["q_WB"][0] = 99
        state["control"]["torque_B"][2] = 99
        state["control"]["thrust"] = 99
        exposed = model.inertia_B
        exposed[0][0] = 99
        self.assertEqual(model.inertia_B[0][0], 0.02)
        with self.assertRaises(AttributeError):
            model.mass = 2
        with self.assertRaises(TypeError):
            csim.DroneData()
        other = csim.DroneModel()
        for call in (lambda: csim.step(other, data), lambda: csim.get_state(other, data),
                     lambda: csim.reset(other, data), lambda: csim.set_control(other, data, 0)):
            with self.assertRaises(ValueError):
                call()
        with self.assertRaises(TypeError):
            csim.step(csim.PendulumModel(), data)
        csim.step(model, data)
        self.assertEqual(csim.get_state(model, independent)["time"], 0)
        self.assertEqual(csim.get_state(model, data)["control"]["thrust"], 12)
        csim.reset(model, data, position_W=[1, 2, 3], q_WB=[-2, 0, 0, 0])
        reset = csim.get_state(model, data)
        self.assertEqual(reset["time"], 0)
        self.assertEqual(reset["control"], {"thrust": 0, "torque_B": [0, 0, 0]})
        self.assertVectorNear(reset["q_WB"], [-1, 0, 0, 0], 0)
        del model, independent
        gc.collect()
        csim.step(data.model, data)
        self.assertAlmostEqual(csim.get_state(data.model, data)["time"], 0.001)

    def test_invalid_inputs_and_atomic_failures(self):
        with self.assertRaises((ValueError, TypeError)):
            csim.make_data(None, position_W=[0,0,0])
        for field in ("mass", "gravity", "timestep"):
            for bad in (0, -1, math.inf, math.nan):
                with self.subTest(field=field, bad=bad), self.assertRaises(ValueError):
                    csim.DroneModel(**{field: bad})
        for bad in ([[0, 0, 0]]*3, [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
                    [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]]):
            with self.assertRaises(ValueError):
                csim.DroneModel(inertia_B=bad)
        with self.assertRaises(TypeError):
            csim.DroneModel(inertia_B=[0.02, 0.02, 0.04])
        model = csim.DroneModel()
        for kwargs in ({"q_WB": [0, 0, 0, 0]}, {"position_W": [math.nan, 0, 0]},
                       {"velocity_W": [0, math.inf, 0]}, {"angular_velocity_B": [0, 0, math.nan]}):
            with self.assertRaises(ValueError):
                csim.make_data(model, **kwargs)
        with self.assertRaises(TypeError):
            csim.make_data(model, q_WB=[1, 0, 0])
        data = csim.make_data(model)
        csim.set_control(model, data, 10, [0, 0, 0.01])
        before = csim.get_state(model, data)
        for bad in (-1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                csim.set_control(model, data, bad)
        with self.assertRaises(ValueError):
            csim.set_control(model, data, 0, [0, math.nan, 0])
        with self.assertRaises(TypeError):
            csim.set_control(model, data, 0, [1, 2])
        with self.assertRaises(OverflowError):
            csim.set_control(model, data, 0, [0, 0, 1e308])
        with self.assertRaises(ValueError):
            csim.reset(model, data, q_WB=[0, 0, 0, 0])
        self.assertEqual(csim.get_state(model, data), before)
        coarse = csim.DroneModel(timestep=1e155)
        data = csim.make_data(coarse, position_W=[0, 0, 10])
        csim.set_control(coarse, data, 11, [0.1, 0.2, 0.3])
        before = csim.get_state(coarse, data)
        with self.assertRaises(OverflowError):
            csim.step(coarse, data)
        self.assertEqual(csim.get_state(coarse, data), before)
        tiny = csim.DroneModel(timestep=5e-324)
        tiny_data = csim.make_data(tiny)
        with self.assertRaises(OverflowError):
            csim.step(tiny, tiny_data)
        self.assertEqual(csim.get_state(tiny, tiny_data)["time"], 0)

    def test_only_physical_api_is_public(self):
        self.assertEqual({name for name in dir(csim) if not name.startswith("_")}, {
            "DroneModel", "DroneData", "PendulumModel", "PendulumData", "PendulumDomainError",
            "SuspendedPayloadModel", "SuspendedPayloadData", "CableDomainError",
            "get_link_state", "get_link_states",
            "DragConfig", "WindField", "get_config", "load_model", "make_data", "step", "set_control", "get_state", "reset",
        })


if __name__ == "__main__":
    unittest.main()
