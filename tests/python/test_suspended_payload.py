"""Coupled physical API: no low-level integrator or algebra bindings."""
import gc
import math
import unittest

import csim


class SuspendedPayloadTests(unittest.TestCase):
    def assertVectorNear(self, actual, expected, tolerance=1e-10):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertTrue(math.isfinite(a))
            self.assertLessEqual(abs(a-b), tolerance)

    def test_hover_and_current_accelerations(self):
        model = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2,
                                          gravity=9.81, timestep=0.005)
        data = csim.make_data(model, thrust=24.525, position_W=[1, 2, 5])
        for _ in range(2000):
            self.assertIsNone(csim.step(model, data))
        state = csim.get_state(model, data)
        self.assertAlmostEqual(state["time"], 10)
        self.assertVectorNear(state["position_W"], [1, 2, 5])
        self.assertVectorNear(state["payload_position_W"], [1, 2, 3.8])
        self.assertVectorNear(state["acceleration_W"], [0, 0, 0])
        self.assertVectorNear(state["payload_acceleration_W"], [0, 0, 0])
        self.assertAlmostEqual(state["tension"], 4.905)
        self.assertEqual(state["mode"], "taut")
        csim.set_control(model, data, thrust=30, torque_B=[0, 0, 0.02])
        new = csim.get_state(model, data)
        self.assertEqual(new["time"], state["time"])
        self.assertVectorNear(new["acceleration_W"], [0, 0, 2.19])
        self.assertVectorNear(new["payload_acceleration_W"], [0, 0, 2.19])
        self.assertVectorNear(new["angular_acceleration_B"], [0, 0, 0.5])
        self.assertVectorNear(new["cable_force_on_drone_W"], [0, 0, -6])
        self.assertVectorNear(new["cable_force_on_payload_W"], [0, 0, 6])
        self.assertAlmostEqual(new["tension"], 6)

    def test_free_orbit_analytic_solution(self):
        model = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2,
                                          gravity=9.81, timestep=0.005)
        data = csim.make_data(model, thrust=0, position_W=[-0.24, 0, 20],
                             velocity_W=[0, -0.48, 0], cable_direction_W=[1, 0, 0],
                             cable_angular_velocity_W=[0, 0, 2])
        initial = csim.get_state(model, data)
        for _ in range(200):
            csim.step(model, data)
        s = csim.get_state(model, data)
        c, v = math.cos(2), math.sin(2)
        self.assertVectorNear(s["cable_direction_W"], [c, v, 0], 1e-10)
        self.assertVectorNear(s["position_W"], [-0.24*c, -0.24*v, 20-4.905], 1e-10)
        self.assertVectorNear(s["payload_position_W"], [0.96*c, 0.96*v, 20-4.905], 1e-10)
        self.assertVectorNear(s["payload_velocity_W"], [-1.92*v, 1.92*c, -9.81], 2e-10)
        self.assertAlmostEqual(s["energy"], initial["energy"], delta=1e-8)
        self.assertAlmostEqual(s["tension"], 1.92)

    def test_snapshot_reset_and_lifetime(self):
        model = csim.SuspendedPayloadModel()
        thrust = (model.drone_mass+model.payload_mass)*model.gravity
        data = csim.make_data(model, thrust=thrust, q_WB=[2, 0, 0, 0])
        independent = csim.make_data(model, thrust=thrust)
        state = csim.get_state(model, data)
        state["cable_direction_W"][2] = 99
        state["payload_position_W"][0] = 99
        state["control"]["thrust"] = 99
        csim.step(model, data)
        self.assertEqual(csim.get_state(model, independent)["time"], 0)
        self.assertIsNone(csim.reset(model, data, thrust=thrust, position_W=[0, 0, 5]))
        reset = csim.get_state(model, data)
        self.assertEqual(reset["time"], 0)
        self.assertVectorNear(reset["cable_direction_W"], [0, 0, -1])
        self.assertVectorNear(reset["payload_position_W"], [0, 0, 4])
        self.assertEqual(reset["control"], {"thrust": thrust, "torque_B": [0, 0, 0]})
        inertia = model.inertia_B
        inertia[0][0] = 99
        self.assertEqual(model.inertia_B[0][0], 0.02)
        for field in ("drone_mass", "payload_mass", "length", "inertia_B", "gravity", "timestep"):
            with self.assertRaises(AttributeError):
                setattr(model, field, 1)
        with self.assertRaises(TypeError):
            csim.SuspendedPayloadData()
        other = csim.SuspendedPayloadModel()
        for call in (lambda: csim.step(other, data), lambda: csim.get_state(other, data),
                     lambda: csim.reset(other, data, thrust), lambda: csim.set_control(other, data, thrust)):
            with self.assertRaises(ValueError):
                call()
        with self.assertRaises(TypeError):
            csim.step(csim.DroneModel(), data)
        del model, independent
        gc.collect()
        csim.step(data.model, data)
        self.assertAlmostEqual(csim.get_state(data.model, data)["time"], 0.001)

    def test_invalid_inputs_and_initial_control(self):
        for field in ("drone_mass", "payload_mass", "length", "gravity", "timestep"):
            for value in (0, -1, math.inf, math.nan):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    csim.SuspendedPayloadModel(**{field: value})
        model = csim.SuspendedPayloadModel()
        with self.assertRaises(TypeError):
            csim.make_data(model)  # Initial control is required, never silently hover.
        with self.assertRaises(csim.CableDomainError):
            csim.make_data(model, thrust=0)
        for kwargs in ({"cable_direction_W": [0, 0, -2]}, {"cable_direction_W": [0, 0, 0]},
                       {"cable_angular_velocity_W": [0, 0, 1]}, {"q_WB": [0, 0, 0, 0]},
                       {"cable_direction_W": [math.nan, 0, -1]}, {"velocity_W": [0, math.inf, 0]}):
            with self.assertRaises(ValueError):
                csim.make_data(model, thrust=12, **kwargs)
        with self.assertRaises(TypeError):
            csim.make_data(model, thrust=12, cable_direction_W=[0, -1])
        with self.assertRaises(OverflowError):
            csim.make_data(model, thrust=12, position_W=[0, 0, 1e20])

    def test_atomic_control_reset_and_step_failures(self):
        model = csim.SuspendedPayloadModel()
        data = csim.make_data(model, thrust=12)
        csim.step(model, data)
        before = csim.get_state(model, data)
        for call in (lambda: csim.set_control(model, data, 0), lambda: csim.reset(model, data, 0)):
            with self.assertRaises(csim.CableDomainError):
                call()
            self.assertEqual(csim.get_state(model, data), before)
        for value in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                csim.set_control(model, data, value)
        with self.assertRaises(OverflowError):
            csim.set_control(model, data, 12, [1e308, 0, 0])
        with self.assertRaises(ValueError):
            csim.reset(model, data, 12, cable_direction_W=[1, 1, 1])
        self.assertEqual(csim.get_state(model, data), before)
        coarse = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2,
                                           gravity=9.81, timestep=0.2)
        boundary = csim.make_data(coarse, thrust=24.525,
                                 cable_direction_W=[math.sin(1.55), 0, -math.cos(1.55)],
                                 cable_angular_velocity_W=[0, -1, 0])
        before = csim.get_state(coarse, boundary)
        with self.assertRaises(csim.CableDomainError):
            csim.step(coarse, boundary)
        self.assertEqual(csim.get_state(coarse, boundary), before)
        for dt in (1e155, 5e-324):
            extreme = csim.SuspendedPayloadModel(timestep=dt)
            data = csim.make_data(extreme, thrust=15)
            before = csim.get_state(extreme, data)
            with self.assertRaises(OverflowError):
                csim.step(extreme, data)
            self.assertEqual(csim.get_state(extreme, data), before)


if __name__ == "__main__":
    unittest.main()
