import unittest

import csim


class SlackTautCableTests(unittest.TestCase):
    def test_hybrid_slack_is_independent_until_recontact(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.01)
        data = csim.make_data(
            model,
            thrust=0.0,
            position_W=[0, 0, 5],
            payload_position_W=[0, 0, 4.5],
            cable_mode="slack",
        )
        for _ in range(20):
            csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertEqual(state["mode"], "slack")
        self.assertAlmostEqual(state["tension"], 0.0)
        self.assertAlmostEqual(state["cable_distance"], 0.5, places=10)

    def test_hybrid_releases_when_tension_becomes_nonpositive(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.01)
        data = csim.make_data(model, thrust=12.0, position_W=[0, 0, 5])
        self.assertGreater(csim.get_state(model, data)["tension"], 0.0)
        csim.set_control(model, data, thrust=0.0)
        csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertEqual(state["mode"], "slack")
        self.assertAlmostEqual(state["tension"], 0.0)

    def test_hybrid_recontact_projects_and_reports_impulse(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.01)
        data = csim.make_data(
            model,
            thrust=30.0,
            position_W=[0, 0, 1],
            payload_position_W=[0, 0, 0.1],
            cable_mode="slack",
        )
        for _ in range(100):
            csim.step(model, data)
            if csim.get_state(model, data)["mode"] == "taut":
                break
        state = csim.get_state(model, data)
        self.assertEqual(state["mode"], "taut")
        self.assertAlmostEqual(state["cable_distance"], model.length, places=10)
        self.assertAlmostEqual(state["cable_radial_velocity"], 0.0, places=10)
        self.assertGreater(sum(x * x for x in state["cable_impulse_W"]), 0.0)

    def test_default_taut_mode_keeps_strict_domain_check(self):
        model = csim.SuspendedPayloadModel(timestep=0.01)
        data = csim.make_data(model, thrust=model.gravity * (model.drone_mass + model.payload_mass), position_W=[0, 0, 5])
        with self.assertRaises(csim.CableDomainError):
            csim.set_control(model, data, thrust=0.0)


if __name__ == "__main__":
    unittest.main()
