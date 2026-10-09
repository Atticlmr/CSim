from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import unittest

import numpy as np

import csim


class DroneBatchTests(unittest.TestCase):
    def assert_snapshot_equal(self, actual, expected):
        for key in ("state", "time", "control"):
            np.testing.assert_array_equal(actual[key], expected[key])

    def test_scalar_parity_and_action_substeps(self):
        for method in ("euler", "midpoint", "heun", "rk4", "dopri5"):
            model = csim.DroneModel(timestep=.002, integrator=method)
            batch = csim.DroneBatch(model, 7, threads=3)
            reference = [csim.make_data(model, position_W=[index, 0, 10],
                                        angular_velocity_B=[.1, .2, .3]) for index in range(7)]
            states = batch.get_state()["state"]
            states[:, 0] = np.arange(7)
            states[:, 2] = 10
            states[:, 10:] = [.1, .2, .3]
            batch.reset(states)
            actions = np.tile([9.8, .001, -.002, .003], (7, 1))
            for _ in range(10):
                self.assertIsNone(batch.step(actions, substeps=3))
                for data in reference:
                    csim.set_control(model, data, thrust=actions[0, 0], torque_B=actions[0, 1:])
                    for _ in range(3):
                        csim.step(model, data)
            actual = batch.get_state()
            for index, data in enumerate(reference):
                expected = csim.get_state(model, data)
                np.testing.assert_array_equal(actual["state"][index],
                    [*expected["position_W"], *expected["velocity_W"],
                     *expected["q_WB"], *expected["angular_velocity_B"]])
                self.assertEqual(actual["time"][index], expected["time"])
            np.testing.assert_array_equal(actual["control"], actions)

    def test_selective_reset_and_snapshot_ownership(self):
        batch = csim.DroneBatch(csim.DroneModel(), 5, threads=2)
        batch.step(np.tile([10, 0, 0, .01], (5, 1)), substeps=5)
        before = batch.get_state()
        states = before["state"][[3, 1]].copy()
        states[:, 2] = [20, 30]
        states[:, 6:10] = [2, 0, 0, 0]
        batch.reset(states, indices=[3, 1])
        after = batch.get_state()
        np.testing.assert_array_equal(after["state"][[3, 1], 2], [20, 30])
        np.testing.assert_array_equal(after["state"][[3, 1], 6], [1, 1])
        np.testing.assert_array_equal(after["time"][[3, 1]], 0)
        np.testing.assert_array_equal(after["control"][[3, 1]], 0)
        for key in before:
            np.testing.assert_array_equal(after[key][[0, 2, 4]], before[key][[0, 2, 4]])
        after["state"][:] = 100
        self.assertEqual(batch.get_state()["state"][3, 2], 20)
        batch.reset(np.empty((0, 13)), indices=[])

    def test_validation_and_whole_batch_rollback(self):
        with self.assertRaises(ValueError):
            csim.DroneBatch(csim.DroneModel(), 0)
        batch = csim.DroneBatch(csim.DroneModel(), 5, threads=2)
        actions = np.tile([10., 0, 0, 0], (5, 1))
        before = batch.get_state()
        for invalid in (np.zeros((4, 4)), np.zeros((5, 3)), np.zeros(20)):
            with self.assertRaises(ValueError):
                batch.step(invalid)
        with self.assertRaises(ValueError):
            batch.step(actions, substeps=0)
        for thrust in (-1, np.nan, np.inf):
            invalid = actions.copy()
            invalid[4, 0] = thrust
            with self.assertRaises(ValueError):
                batch.step(invalid)
        states = before["state"][:2].copy()
        for indices, error in (([1, 1], ValueError), ([1, 5], IndexError), ([1], ValueError)):
            with self.assertRaises(error):
                batch.reset(states, indices=indices)
        states[1, 6:10] = 0
        with self.assertRaises(ValueError):
            batch.reset(states, indices=[0, 4])
        self.assert_snapshot_equal(batch.get_state(), before)

        model = csim.DroneModel(timestep=1e155)
        failing = csim.DroneBatch(model, 5, threads=2)
        before = failing.get_state()
        actions[:] = [model.gravity, 0, 0, 0]
        actions[4] = [11, .1, .2, .3]
        with self.assertRaises(OverflowError):
            failing.step(actions)
        self.assert_snapshot_equal(failing.get_state(), before)
        actions[4] = [model.gravity, 0, 0, 0]
        failing.step(actions)
        np.testing.assert_array_equal(failing.get_state()["time"], 1e155)

    def test_array_conversion_and_concurrent_access(self):
        batch = csim.DroneBatch(csim.DroneModel(), 7, threads=3)
        backing = np.zeros((7, 8), dtype=np.float32)
        backing[:, 0] = 10
        actions = backing[:, ::2]
        self.assertFalse(actions.flags.c_contiguous)
        batch.step(actions)
        np.testing.assert_array_equal(batch.get_state()["control"], actions)

        def advance():
            for _ in range(20):
                batch.step(actions, substeps=2)
                self.assertEqual(batch.num_envs, 7)
                snapshot = batch.get_state()
                np.testing.assert_array_equal(snapshot["time"], snapshot["time"][0])

        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [executor.submit(advance) for _ in range(2)]
            for future in futures:
                future.result(timeout=30)
        time = 0
        for _ in range(81):
            time += .001
        np.testing.assert_array_equal(batch.get_state()["time"], time)
        self.assertEqual(batch.threads, 3)
        self.assertIsInstance(batch.model, csim.DroneModel)

    def test_imported_pose_and_wind_match_single_environment(self):
        path = Path(__file__).resolve().parents[1] / "fixtures/model_import/fixed_frame.xml"
        model = csim.load_model(str(path), free_base=True,
                                drone_drag=csim.DragConfig(k1=.1),
                                wind=csim.WindField(velocity_W=[.2, -.3, .1]))
        reference = csim.make_data(model)
        batch = csim.DroneBatch(model, 3, threads=2)
        self.assertIs(batch.model, model)
        del model
        for _ in range(2):
            expected = csim.get_state(batch.model, reference)
            state = [*expected["position_W"], *expected["velocity_W"],
                     *expected["q_WB"], *expected["angular_velocity_B"]]
            np.testing.assert_array_equal(batch.get_state()["state"], np.tile(state, (3, 1)))
            actions = np.tile([10., .001, 0, 0], (3, 1))
            batch.step(actions)
            csim.set_control(batch.model, reference, thrust=10., torque_B=[.001, 0, 0])
            csim.step(batch.model, reference)


if __name__ == "__main__":
    unittest.main()
