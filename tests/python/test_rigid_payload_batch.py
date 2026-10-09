from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import unittest

import numpy as np

import csim


ROOT = Path(__file__).resolve().parents[2] / "examples/models/rigid_payload"


class RigidPayloadBatchTests(unittest.TestCase):
    def test_reset_mode_validation_and_whole_batch_atomicity(self):
        model=self.load()
        batch=csim.RigidPayloadBatch(model,3,threads=2)
        batch.step(np.tile([15.,0,0,0],(3,1)))
        before=batch.get_state()
        for flag in (float('nan'),float('inf'),float('-inf'),-1.,.5,2.):
            invalid=before['state'].copy()
            invalid[2,26]=flag
            with self.assertRaisesRegex(ValueError,'Slack flag'): batch.reset(invalid)
            after=batch.get_state()
            for key in ('state','control','time','event_count','event_energy_loss','event_time'):
                np.testing.assert_array_equal(after[key],before[key])
            self.assertEqual(after['event_type'],before['event_type'])
        reference=csim.get_state(model,csim.make_data(model))
        self.assertEqual(reference['mode'],'slack')
        batch.reset(np.tile(self.flatten(reference),(3,1)))
        np.testing.assert_array_equal(batch.get_state()['state'][:,26],1.)
        config=csim.get_config(model)
        config['cable_mode']='taut'
        strict=csim.RigidPayloadModel.from_config(config)
        strict_batch=csim.RigidPayloadBatch(strict,2,threads=2)
        states=strict_batch.get_state()['state']
        states[1,26]=1
        with self.assertRaisesRegex(ValueError,'Slack state'): strict_batch.reset(states)
        np.testing.assert_array_equal(strict_batch.get_state()['state'][:,26],0.)
        strict_batch.step(np.tile([15.,0,0,0],(2,1)))

    def load(self, **kwargs):
        return csim.load_suspended_model(
            str(ROOT / "drone.urdf"), str(ROOT / "cable.json"), str(ROOT / "payload.urdf"), **kwargs
        )

    @staticmethod
    def flatten(state):
        return np.asarray(
            list(state["position_W"]) + list(state["velocity_W"]) + list(state["q_WB"]) +
            list(state["angular_velocity_B"]) + list(state["payload_position_W"]) +
            list(state["payload_velocity_W"]) + list(state["payload_q_WP"]) +
            list(state["payload_angular_velocity_P"]) + [1 if state["mode"] == "slack" else 0],
            dtype=float,
        )

    def test_shape_scalar_parity_and_events(self):
        model = self.load(timestep=.002)
        batch = csim.RigidPayloadBatch(model, 4, threads=2)
        states = batch.get_state()
        self.assertEqual(states["state"].shape, (4, 27))
        self.assertEqual(states["control"].shape, (4, 4))
        np.testing.assert_array_equal(states["state"][:, 26], 1)

        initial = np.tile(states["state"][0], (4, 1))
        initial[:, 0] = [0, .01, .02, .03]
        initial[:, 13] = initial[:, 0]
        batch.reset(initial)
        controls = np.tile([15., 0, 0, .001], (4, 1))
        scalar = []
        for index in range(4):
            data = csim.make_data(model, thrust=0, mode="slack",
                                  position_W=initial[index, :3],
                                  payload_position_W=initial[index, 13:16])
            scalar.append(data)
        for _ in range(5):
            batch.step(controls, substeps=2)
            for data in scalar:
                csim.set_control(model, data, thrust=15, torque_B=[0, 0, .001])
                csim.step(model, data)
                csim.step(model, data)
        actual = batch.get_state()
        for index, data in enumerate(scalar):
            reference = csim.get_state(model, data)
            np.testing.assert_allclose(actual["state"][index], self.flatten(reference), rtol=0, atol=1e-12)
            np.testing.assert_allclose(actual["time"][index], reference["time"])
            self.assertEqual(actual["event_count"][index], 0)

        event_model = self.load(timestep=.02)
        event_batch = csim.RigidPayloadBatch(event_model, 2, threads=2)
        event_states = event_batch.get_state()["state"]
        event_states[:, 26] = 1
        event_states[:, 15] = -1.19
        event_states[:, 18] = -1
        event_batch.reset(event_states)
        event_batch.step(np.zeros((2, 4)))
        events = event_batch.get_state()
        np.testing.assert_array_equal(events["event_count"], [1, 1])
        self.assertEqual(events["event_type"][0][0], "impact")
        self.assertTrue(np.all(np.isfinite(events["event_energy_loss"])))

    def test_selective_reset_failure_and_thread_safety(self):
        model = self.load()
        batch = csim.RigidPayloadBatch(model, 6, threads=3)
        actions = np.tile([15., 0, 0, 0], (6, 1))
        batch.step(actions)
        before = batch.get_state()
        states = before["state"][[1, 4]].copy()
        states[:, 2] = [10, 20]
        states[:, 15] += states[:, 2] - before["state"][[1, 4], 2]
        batch.reset(states, indices=[4, 1])
        after = batch.get_state()
        np.testing.assert_array_equal(after["state"][[4, 1], 2], [10, 20])
        np.testing.assert_array_equal(after["time"][[4, 1]], 0)
        np.testing.assert_array_equal(after["state"][[0, 2, 3, 5]], before["state"][[0, 2, 3, 5]])

        invalid = actions.copy()
        invalid[5, 0] = -1
        with self.assertRaises(ValueError):
            batch.step(invalid)
        current = batch.get_state()
        np.testing.assert_array_equal(current["state"], after["state"])
        with self.assertRaises(ValueError):
            batch.reset(np.zeros((2, 27)), indices=[1, 1])
        with self.assertRaises(IndexError):
            batch.reset(np.zeros((1, 27)), indices=[6])
        batch.reset(current["state"])

        def advance():
            for _ in range(10):
                batch.step(actions, substeps=2)
                snapshot = batch.get_state()
                self.assertTrue(np.all(snapshot["time"] == snapshot["time"][0]))

        with ThreadPoolExecutor(max_workers=2) as executor:
            first, second = executor.submit(advance), executor.submit(advance)
            first.result(timeout=30)
            second.result(timeout=30)
        np.testing.assert_allclose(batch.get_state()["time"], .04, rtol=0, atol=1e-15)

    def test_control_release_events_match_scalar_and_clear_after_substeps(self):
        config = csim.get_config(self.load())
        config["drone_attachment_B"] = [0, 0, 0]
        config["payload_attachment_P"] = [0, 0, 0]
        for threads in (1, 3):
            with self.subTest(threads=threads):
                model = csim.RigidPayloadModel.from_config(config)
                batch = csim.RigidPayloadBatch(model, 6, threads=threads)
                hover = (model.drone_mass + model.payload_mass) * model.gravity
                batch.step(np.tile([hover, 0, 0, 0], (6, 1)))
                before = batch.get_state()
                np.testing.assert_array_equal(before["state"][:, 26], 0)
                scalar = csim.make_data(model, thrust=hover)
                csim.step(model, scalar)
                csim.step(model, scalar, thrust=0, torque_B=[0, 0, 0])
                reference = csim.get_state(model, scalar)
                batch.step(np.zeros((6, 4)), substeps=3)
                after = batch.get_state()
                np.testing.assert_array_equal(after["state"][:, 26], 1)
                np.testing.assert_array_equal(after["event_count"], 1)
                for index in range(6):
                    self.assertEqual(after["event_type"][index], ["release"])
                    self.assertEqual(after["event_time"][index], reference["cable_events"][0]["time"])
                    self.assertEqual(after["event_time"][index], before["time"][index])
                batch.step(np.zeros((6, 4)))
                np.testing.assert_array_equal(batch.get_state()["event_count"], 0)


if __name__ == "__main__":
    unittest.main()
