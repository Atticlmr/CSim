import json
import math
from pathlib import Path
import tempfile
import unittest

import csim


EXAMPLES = Path(__file__).resolve().parents[2] / "examples/models/rigid_payload"


class RigidPayloadTests(unittest.TestCase):
    def load(self, drone="urdf", payload=None, **kwargs):
        return csim.load_suspended_model(
            str(EXAMPLES / f"drone.{drone}"), str(EXAMPLES / "cable.json"),
            str(EXAMPLES / f"payload.{payload or drone}"), **kwargs,
        )

    def near(self, actual, expected, tolerance=1e-11):
        self.assertEqual(len(actual), len(expected))
        for value, reference in zip(actual, expected):
            self.assertTrue(math.isfinite(value))
            self.assertLessEqual(abs(value - reference), tolerance)

    def test_three_formats_and_mixed_body_files(self):
        reference = None
        for drone, payload in (("urdf", "urdf"), ("json", "json"), ("xml", "xml"), ("urdf", "xml")):
            model = self.load(drone, payload)
            self.assertIsInstance(model, csim.RigidPayloadModel)
            self.assertEqual(model.drone_mass, 1)
            self.assertEqual(model.payload_mass, .5)
            self.near(model.drone_attachment_B, [.1, 0, -.1])
            self.near(model.payload_attachment_P, [.1, 0, .1])
            data = csim.make_data(model, thrust=15)
            state = csim.get_state(model, data)
            self.assertAlmostEqual(state["tension"], 15 / (3 + .01/.02 + .01/.015))
            self.near(state["position_W"], [.05, 0, 0])
            self.assertEqual(state["mode"], "taut")
            self.assertGreater(state["angular_acceleration_B"][1], 0)
            self.assertLess(state["payload_angular_acceleration_P"][1], 0)
            for _ in range(200):
                csim.step(model, data)
            state = csim.get_state(model, data)
            self.assertAlmostEqual(state["cable_distance"], 1, places=11)
            self.assertAlmostEqual(state["cable_radial_velocity"], 0, places=11)
            self.assertAlmostEqual(sum(value*value for value in state["payload_q_WP"]), 1)
            if reference is None:
                reference = state
            else:
                self.assertEqual(state, reference)

    def test_link_snapshots_and_root_axis_conversion(self):
        rotation = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        model = self.load(drone_root_to_body=rotation, payload_root_to_body=rotation)
        self.near(model.drone_attachment_B, [0, .1, -.1])
        self.near(model.payload_attachment_P, [0, .1, .1])
        data = csim.make_data(model, thrust=15)
        state = csim.get_state(model, data)
        links = csim.get_link_states(model, data)
        self.assertEqual(set(links), set(model.link_names))
        for body in ("drone", "payload"):
            key = f"{body}/cable_attachment"
            self.near(links[key]["position_W"], state[f"{body}_attachment_position_W"])
            self.near(links[key]["velocity_W"], state[f"{body}_attachment_velocity_W"])
            self.assertEqual(links[key]["parent"], f"{body}/base")
            self.assertEqual(links[key], csim.get_link_state(model, data, key))
        with self.assertRaises(KeyError):
            csim.get_link_state(model, data, "cable_attachment")

    def test_independent_states_reset_and_atomic_validation(self):
        model = self.load()
        data = csim.make_data(model, thrust=15, position_W=[0, 0, 3], q_WB=[2, 0, 0, 0])
        independent = csim.make_data(model)
        self.assertEqual(csim.get_state(model, independent)["mode"], "slack")
        csim.step(model, data)
        self.assertEqual(csim.get_state(model, independent)["time"], 0)
        before = csim.get_state(model, data)
        for fields in ({"q_WB": [0, 0, 0, 0]}, {"payload_q_WP": [0, 0, 0, 0]},
                       {"payload_position_W": [0, 0, 100]}, {"velocity_W": [math.nan, 0, 0]},
                       {"unknown": 1}, {"mode": "hybrid"}, {"payload_velocity_W": [0, 0, 1]}):
            with self.subTest(fields=fields), self.assertRaises((ValueError, TypeError)):
                csim.reset(model, data, thrust=15, **fields)
            self.assertEqual(csim.get_state(model, data), before)
        with self.assertRaises(ValueError):
            csim.set_control(model, data, thrust=-1)
        self.assertEqual(csim.get_state(model, data), before)
        other = self.load()
        for operation in (lambda: csim.get_state(other, data), lambda: csim.step(other, data),
                          lambda: csim.reset(other, data), lambda: csim.set_control(other, data, 15)):
            with self.assertRaises(ValueError):
                operation()
        csim.reset(model, data, thrust=15, payload_q_WP=[2, 0, 0, 0])
        self.assertEqual(csim.get_state(model, data)["time"], 0)
        self.assertEqual(data.model, model)

    def test_json_validation_and_fixed_marker_name(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "body.json"
            original = json.loads((EXAMPLES / "drone.json").read_text())
            invalid = [dict(original, mass=0), dict(original, mass=True), dict(original, unknown=1),
                       dict(original, inertia=[[1, 0, 0]]), dict(original, cable_attachment=[0, 0]),
                       dict(original, center_of_mass=[0, 0, math.inf])]
            missing = dict(original)
            del missing["cable_attachment"]
            invalid.append(missing)
            for config in invalid:
                path.write_text(json.dumps(config))
                with self.assertRaises((ValueError, TypeError)):
                    csim.load_suspended_model(str(path), str(EXAMPLES / "cable.json"), str(EXAMPLES / "payload.urdf"))
            path.write_text('{"name":"a", "name":"b"}')
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                csim.load_suspended_model(str(path), str(EXAMPLES / "cable.json"), str(EXAMPLES / "payload.urdf"))
            path = Path(directory) / "body.urdf"
            path.write_text((EXAMPLES / "drone.urdf").read_text().replace("cable_attachment", "other_name"))
            with self.assertRaisesRegex(ValueError, "cable_attachment"):
                csim.load_suspended_model(str(path), str(EXAMPLES / "cable.json"), str(EXAMPLES / "payload.urdf"))

    def test_cable_validation_and_strict_taut_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cable.json"
            for config in ({}, {"length": -1}, {"length": 1, "drone_attachment": "base"},
                           {"length": 1, "initial_direction_W": [0, 0, -2]}, {"length": 1, "mode": "elastic"},
                           {"length": 1, "max_events": 1.5}):
                path.write_text(json.dumps(config))
                with self.assertRaises(ValueError):
                    csim.load_suspended_model(str(EXAMPLES / "drone.urdf"), str(path), str(EXAMPLES / "payload.urdf"))
            path.write_text('{"length": 1, "mode": "taut"}')
            model = csim.load_suspended_model(str(EXAMPLES / "drone.urdf"), str(path), str(EXAMPLES / "payload.urdf"))
            with self.assertRaises(csim.CableDomainError):
                csim.make_data(model)
            with self.assertRaises(ValueError):
                csim.make_data(model, mode="slack")
            data = csim.make_data(model, thrust=15)
            before = csim.get_state(model, data)
            with self.assertRaises(csim.CableDomainError):
                csim.set_control(model, data, 0)
            self.assertEqual(csim.get_state(model, data), before)

    def test_hybrid_impact_angular_response_and_step_rollback(self):
        model = self.load(timestep=.02)
        data = csim.make_data(model, mode="slack", payload_position_W=[.05, 0, -1.19], payload_velocity_W=[0, 0, -1])
        csim.step(model, data)
        state = csim.get_state(model, data)
        event = state["cable_events"][0]
        self.assertEqual(event["type"], "impact")
        self.assertAlmostEqual(event["time"], .01, places=8)
        self.assertAlmostEqual(event["radial_velocity_after"], 0)
        self.assertGreater(abs(state["payload_angular_velocity_P"][1]), 0)
        model = self.load(timestep=.02, max_substeps=1)
        data = csim.make_data(model, mode="slack", payload_velocity_W=[0, 0, -1])
        before = csim.get_state(model, data)
        with self.assertRaises(RuntimeError):
            csim.step(model, data)
        self.assertEqual(csim.get_state(model, data), before)

    def test_general_integrators_and_config_snapshot(self):
        for method in ("euler", "midpoint", "heun", "rk4", "dopri5"):
            model = self.load(integrator=method)
            data = csim.make_data(model, thrust=15)
            for _ in range(20):
                csim.step(model, data)
            state = csim.get_state(model, data)
            self.assertAlmostEqual(state["cable_distance"], 1, places=11)
            self.assertAlmostEqual(state["cable_radial_velocity"], 0, places=11)
            config = csim.get_config(model)
            self.assertEqual(config["kind"], "rigid_payload")
            self.assertEqual(config["integrator"], method)
            config["payload_inertia_P"][0][0] = 99
            self.assertEqual(model.payload_inertia_P[0][0], .01)

    def test_fixed_tree_attachment_follows_aggregate_center(self):
        fixture = Path(__file__).resolve().parents[1] / "fixtures/model_import/fixed_frame.urdf"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tree.urdf"
            marker = '''<link name="cable_attachment"/>
                <joint name="cable_mount" type="fixed"><parent link="arm"/>
                <child link="cable_attachment"/><origin xyz="0.1 0.2 0.3"/></joint>'''
            path.write_text(fixture.read_text().replace("</robot>", marker + "</robot>"))
            model = csim.load_suspended_model(str(path), str(EXAMPLES / "cable.json"), str(EXAMPLES / "payload.urdf"))
            self.assertEqual(model.drone_mass, 2.5)
            self.near(model.drone_attachment_B, [.06, .1, .38])
            for actual, expected in zip(model.inertia_B, [[.036, 0, -.008], [0, .041, 0], [-.008, 0, .059]]):
                self.near(actual, expected)

    def test_control_release_survives_step_once_and_reset_clears_it(self):
        config = csim.get_config(self.load())
        config["drone_attachment_B"] = [0, 0, 0]
        config["payload_attachment_P"] = [0, 0, 0]
        for method in ("euler", "rk4", "dopri5", "bdf2", "lie_rk4"):
            for atomic in (False, True):
                with self.subTest(method=method, atomic=atomic):
                    config["integrator"] = method
                    model = csim.RigidPayloadModel.from_config(config)
                    hover = (model.drone_mass + model.payload_mass) * model.gravity
                    data = csim.make_data(model, thrust=hover)
                    csim.step(model, data)
                    release_time = csim.get_state(model, data)["time"]
                    if atomic:
                        csim.step(model, data, thrust=0, torque_B=[0, 0, 0])
                    else:
                        csim.set_control(model, data, thrust=0)
                        csim.set_control(model, data, thrust=0)
                        self.assertEqual(len(csim.get_state(model, data)["cable_events"]), 1)
                        csim.step(model, data)
                    state = csim.get_state(model, data)
                    self.assertEqual(state["mode"], "slack")
                    self.assertEqual(len(state["cable_events"]), 1)
                    event = state["cable_events"][0]
                    self.assertEqual(event["type"], "release")
                    self.assertEqual(event["time"], release_time)
                    self.assertEqual(event["impulse_W"], [0, 0, 0])
                    self.assertEqual(event["energy_loss"], 0)
                    csim.step(model, data)
                    self.assertEqual(csim.get_state(model, data)["cable_events"], [])
                    csim.reset(model, data, thrust=hover)
                    csim.set_control(model, data, thrust=0)
                    csim.reset(model, data, thrust=hover)
                    csim.step(model, data)
                    self.assertEqual(csim.get_state(model, data)["cable_events"], [])

    def test_pending_release_order_event_budget_and_retry(self):
        config = csim.get_config(self.load())
        config["drone_attachment_B"] = [0, 0, 0]
        config["payload_attachment_P"] = [0, 0, 0]
        for limit in (1, 2):
            with self.subTest(limit=limit):
                config["max_events"] = limit
                model = csim.RigidPayloadModel.from_config(config)
                data = csim.make_data(model, thrust=15)
                csim.set_control(model, data, thrust=0)
                csim.set_control(model, data, thrust=15)
                before = csim.get_state(model, data)
                if limit == 1:
                    with self.assertRaisesRegex(RuntimeError, "event limit"):
                        csim.step(model, data)
                    self.assertEqual(csim.get_state(model, data), before)
                    csim.set_control(model, data, thrust=0)
                    csim.step(model, data)
                    self.assertEqual([event["type"] for event in csim.get_state(model, data)["cable_events"]], ["release"])
                else:
                    csim.step(model, data)
                    events = csim.get_state(model, data)["cable_events"]
                    self.assertEqual([event["type"] for event in events], ["release", "impact"])
                    self.assertEqual([event["time"] for event in events], [0, 0])
                csim.step(model, data)
                self.assertEqual(csim.get_state(model, data)["cable_events"], [])

    def test_failed_release_step_preserves_pending_events_and_control(self):
        model = self.load(timestep=.02, max_substeps=1)
        data = csim.make_data(model, thrust=15)
        before = csim.get_state(model, data)
        with self.assertRaises(RuntimeError):
            csim.step(model, data, thrust=0, torque_B=[0, 0, 0])
        self.assertEqual(csim.get_state(model, data), before)
        csim.set_control(model, data, thrust=0)
        before = csim.get_state(model, data)
        with self.assertRaises(RuntimeError):
            csim.step(model, data)
        self.assertEqual(csim.get_state(model, data), before)

    def test_from_config_matches_file_inertia_validation(self):
        config = csim.get_config(self.load())
        invalid_tensors = (
            [[.01, 0, 0], [0, .01, 0], [0, 0, 1]],
            [[.505, -.495, 0], [-.495, .505, 0], [0, 0, .01]],
        )
        with tempfile.TemporaryDirectory() as directory:
            for body, field in (("drone", "inertia_B"), ("payload", "payload_inertia_P")):
                for scale in (1e-12, 1, 1e12):
                    for tensor in invalid_tensors:
                        with self.subTest(body=body, scale=scale, tensor=tensor):
                            scaled = [[value * scale for value in row] for row in tensor]
                            restored_config = dict(config, **{field: scaled})
                            with self.assertRaisesRegex(ValueError, "principal-moment triangle"):
                                csim.RigidPayloadModel.from_config(restored_config)
                            description = json.loads((EXAMPLES / f"{body}.json").read_text())
                            description["inertia"] = scaled
                            path = Path(directory) / f"{body}.json"
                            path.write_text(json.dumps(description))
                            drone = path if body == "drone" else EXAMPLES / "drone.json"
                            payload = path if body == "payload" else EXAMPLES / "payload.json"
                            with self.assertRaisesRegex(ValueError, "principal-moment triangle"):
                                csim.load_suspended_model(str(drone), str(EXAMPLES / "cable.json"), str(payload))
            for field in ("inertia_B", "payload_inertia_P"):
                for tensor in ([[.01, 0, 0], [0, .01, 0], [0, 0, .02]],
                               [[.015, -.005, 0], [-.005, .015, 0], [0, 0, .01]]):
                    restored_config = dict(config, **{field: tensor})
                    restored = csim.RigidPayloadModel.from_config(restored_config)
                    self.assertEqual(csim.get_config(restored), restored_config)

    def test_initial_root_pose_and_independent_payload_orientation(self):
        with tempfile.TemporaryDirectory() as directory:
            drone = json.loads((EXAMPLES / "drone.json").read_text())
            payload = json.loads((EXAMPLES / "payload.json").read_text())
            half = math.sqrt(.5)
            drone["initial_pose"] = {"position_W": [1, 2, 3], "q_WR": [half, 0, 0, half]}
            payload["initial_pose"] = {"position_W": [100, 200, 300], "q_WR": [half, half, 0, 0]}
            drone_path, payload_path = Path(directory) / "drone.json", Path(directory) / "payload.json"
            drone_path.write_text(json.dumps(drone))
            payload_path.write_text(json.dumps(payload))
            model = csim.load_suspended_model(str(drone_path), str(EXAMPLES / "cable.json"), str(payload_path))
            data = csim.make_data(model, thrust=15)
            state = csim.get_state(model, data)
            self.near(state["position_W"], [1, 2.05, 3])
            self.near(state["payload_q_WP"], [half, half, 0, 0])
            self.near(state["drone_attachment_position_W"], [1, 2.15, 2.9])
            self.near(state["payload_position_W"], [.9, 2.25, 1.9])

    def test_mjcf_site_validation_and_ambiguous_attachment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "drone.xml"
            original = (EXAMPLES / "drone.xml").read_text()
            invalid = [original.replace('size="0.005"', 'size="-1"'),
                       original.replace('size="0.005"', 'size="0.1 0.2"'),
                       original.replace('size="0.005"', 'group="1.5"'),
                       original.replace('size="0.005"', 'type="mesh"'),
                       original.replace('</body>', '<site name="cable_attachment"/></body>'),
                       original.replace('</body>', '''<body name="cable_attachment">
                           <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
                           </body></body>''')]
            for source in invalid:
                path.write_text(source)
                with self.assertRaises(ValueError):
                    csim.load_suspended_model(str(path), str(EXAMPLES / "cable.json"), str(EXAMPLES / "payload.urdf"))

    def test_unpowered_spatial_motion_conserves_energy_and_linear_momentum(self):
        model = self.load(timestep=.002)
        data = csim.make_data(model, position_W=[0, 0, 3], payload_velocity_W=[.8, 0, 0],
                              angular_velocity_B=[.1, 0, .2], payload_angular_velocity_P=[-.1, 0, .1])
        initial = csim.get_state(model, data)
        loss = 0
        for _ in range(500):
            csim.step(model, data)
            state = csim.get_state(model, data)
            loss += state["impact_energy_loss"]
            self.assertLessEqual(state["cable_distance"], model.length + 1e-10)
            if state["mode"] == "taut":
                self.assertAlmostEqual(state["cable_radial_velocity"], 0, places=10)
            self.assertAlmostEqual(sum(value*value for value in state["payload_q_WP"]), 1)
        self.assertAlmostEqual(state["energy"] + loss, initial["energy"], places=9)
        for axis in range(3):
            initial_momentum = model.drone_mass*initial["velocity_W"][axis] + model.payload_mass*initial["payload_velocity_W"][axis]
            final_momentum = model.drone_mass*state["velocity_W"][axis] + model.payload_mass*state["payload_velocity_W"][axis]
            gravity_impulse = -(model.drone_mass + model.payload_mass)*model.gravity*state["time"] if axis == 2 else 0
            self.assertAlmostEqual(final_momentum, initial_momentum + gravity_impulse, places=10)


if __name__ == "__main__":
    unittest.main()
