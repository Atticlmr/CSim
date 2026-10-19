from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import random
import unittest

import numpy as np

import csim
from csim_experiments import BatchRandomizer, RandomStreams


SETTINGS = dict(position_low=[-1, -1, 1], position_high=[1, 1, 2],
                velocity_low=[-.1, -.1, -.1], velocity_high=[.1, .1, .1])


class RandomStreamsTests(unittest.TestCase):
    def test_reseed_repeats_samples_and_different_seeds_differ(self):
        streams = RandomStreams(6, seed=42)
        uniform = streams.uniform(-1, 1, size=(2, 3))
        normal = streams.normal(scale=.1, size=3)
        streams.seed(42)
        np.testing.assert_array_equal(streams.uniform(-1, 1, size=(2, 3)), uniform)
        np.testing.assert_array_equal(streams.normal(scale=.1, size=3), normal)
        streams.seed(43)
        self.assertFalse(np.array_equal(streams.uniform(-1, 1, size=(2, 3)), uniform))
        self.assertFalse(np.array_equal(uniform[0], uniform[1]))

    def test_environment_streams_ignore_batch_size_and_sampling_order(self):
        whole = RandomStreams(6, seed=2718)
        subset = RandomStreams(10, seed=2718)
        for method in ("uniform", "normal"):
            expected = getattr(whole, method)(size=4)
            getattr(subset, method)(size=100, indices=[9, 7])
            actual = getattr(subset, method)(size=4, indices=[4, 1, 3])
            np.testing.assert_array_equal(actual, expected[[4, 1, 3]])
            untouched = getattr(subset, method)(size=4, indices=[0, 2, 5])
            np.testing.assert_array_equal(untouched, expected[[0, 2, 5]])

    def test_entropy_metadata_and_global_rng_isolation(self):
        python_before = random.getstate()
        numpy_before = np.random.get_state()
        streams = RandomStreams(3)
        metadata = streams.metadata
        json.dumps(metadata)
        self.assertIsInstance(metadata["seed"], int)
        self.assertTrue(0 <= metadata["seed"] < 2**128)
        expected = streams.normal(size=5)
        repeated = RandomStreams(3, seed=metadata["seed"])
        np.testing.assert_array_equal(repeated.normal(size=5), expected)
        metadata["seed"] = -1
        self.assertGreaterEqual(streams.metadata["seed"], 0)
        self.assertEqual(random.getstate(), python_before)
        numpy_after = np.random.get_state()
        self.assertEqual(numpy_before[0], numpy_after[0])
        np.testing.assert_array_equal(numpy_before[1], numpy_after[1])
        self.assertEqual(numpy_before[2:], numpy_after[2:])

    def test_invalid_inputs_and_overflow_do_not_consume_streams(self):
        streams = RandomStreams(3, seed=np.int64(0))
        reference = RandomStreams(3, seed=0)
        for seed in (-1, 2**128, True, np.bool_(False), 1.5, "42"):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                streams.seed(seed)
        for indices, error in (([1, 1], ValueError), ([-1], IndexError), ([3], IndexError),
                               ([.5], ValueError), ([True], ValueError), (1, ValueError)):
            with self.subTest(indices=indices), self.assertRaises(error):
                streams.uniform(indices=indices)
        for size in (-1, 1.5, True, (2, -.5)):
            with self.subTest(size=size), self.assertRaises(ValueError):
                streams.normal(size=size)
        for operation in (lambda: streams.uniform(2, 1), lambda: streams.uniform(-1e308, 1e308),
                          lambda: streams.normal(scale=-1), lambda: streams.normal(loc=np.nan)):
            with self.assertRaises(ValueError):
                operation()
        with self.assertRaises(OverflowError):
            streams.normal(loc=1e308, scale=1e308, size=100)
        self.assertEqual(streams.uniform(indices=[]).shape, (0,))
        self.assertEqual(streams.normal(size=0).shape, (3, 0))
        np.testing.assert_array_equal(streams.uniform(size=5), reference.uniform(size=5))

    def test_concurrent_sampling_serializes_rng_updates(self):
        streams = RandomStreams(1, seed=123)
        reference = RandomStreams(1, seed=123)
        expected = [tuple(reference.uniform(size=3)[0]) for _ in range(30)]
        with ThreadPoolExecutor(max_workers=4) as pool:
            actual = list(pool.map(lambda _: tuple(streams.uniform(size=3)[0]), range(30)))
        self.assertEqual(sorted(actual), sorted(expected))
        np.testing.assert_array_equal(streams.normal(size=5), reference.normal(size=5))


class BatchRandomizerTests(unittest.TestCase):
    def make_batch(self, kind, count=6, threads=2):
        if kind == "drone":
            return csim.DroneBatch(csim.DroneModel(), count, threads=threads)
        root = Path(__file__).resolve().parents[2] / "examples/models/rigid_payload"
        model = csim.load_suspended_model(str(root / "drone.urdf"), str(root / "cable.json"),
                                          str(root / "payload.urdf"))
        return csim.RigidPayloadBatch(model, count, threads=threads)

    def test_reseed_reproduces_initial_states_and_parallel_trajectories(self):
        for kind in ("drone", "rigid"):
            with self.subTest(kind=kind):
                serial = self.make_batch(kind, threads=1)
                parallel = self.make_batch(kind, threads=3)
                first = BatchRandomizer(serial, seed=42)
                second = BatchRandomizer(parallel, seed=42)
                first.reset(**SETTINGS)
                second.reset(**SETTINGS)
                initial = serial.get_state()["state"]
                np.testing.assert_array_equal(initial, parallel.get_state()["state"])
                actions = np.tile([15, 0, 0, .001], (6, 1))
                for _ in range(5):
                    serial.step(actions, substeps=2)
                    parallel.step(actions, substeps=2)
                np.testing.assert_array_equal(serial.get_state()["state"], parallel.get_state()["state"])
                first.reset(seed=42, **SETTINGS)
                np.testing.assert_array_equal(serial.get_state()["state"], initial)
                first.reset(**SETTINGS)
                self.assertFalse(np.array_equal(serial.get_state()["state"], initial))

    def test_selective_reset_order_and_other_streams_are_independent(self):
        for kind in ("drone", "rigid"):
            with self.subTest(kind=kind):
                whole, selected = self.make_batch(kind), self.make_batch(kind)
                first, second = BatchRandomizer(whole, seed=17), BatchRandomizer(selected, seed=17)
                first.reset(**SETTINGS)
                second.reset(indices=[4, 1], **SETTINGS)
                second.reset(indices=[0, 2, 3, 5], **SETTINGS)
                np.testing.assert_array_equal(whole.get_state()["state"], selected.get_state()["state"])
                first.reset(indices=[1, 4], **SETTINGS)
                second.reset(indices=[4, 1], **SETTINGS)
                np.testing.assert_array_equal(whole.get_state()["state"], selected.get_state()["state"])
                first.reset(indices=[], **SETTINGS)
                np.testing.assert_array_equal(first.random.uniform(size=3), second.random.uniform(size=3))

    def test_rigid_offsets_preserve_geometry_and_relative_velocity(self):
        batch = self.make_batch("rigid")
        base = batch.get_state()["state"]
        randomizer = BatchRandomizer(batch, seed=23, base_state=base)
        base[:] = 99
        randomizer.reset(**SETTINGS)
        state = batch.get_state()["state"]
        np.testing.assert_allclose(state[:, 13:16] - state[:, :3], np.tile([0, 0, -1.2], (6, 1)), atol=1e-15)
        np.testing.assert_array_equal(state[:, 16:19], state[:, 3:6])
        np.testing.assert_array_equal(state[:, 6:10], np.tile([1, 0, 0, 0], (6, 1)))
        np.testing.assert_array_equal(state[:, 19:23], state[:, 6:10])
        self.assertTrue(np.all(state[:, 2] >= 1))
        self.assertTrue(np.all(state[:, 2] < 2))
        batch.step(np.tile([15, 0, 0, 0], (6, 1)))
        self.assertTrue(np.isfinite(batch.get_state()["state"]).all())

    def test_failed_physical_reset_rolls_back_rng_and_seed(self):
        model = csim.DroneModel(gravity=1e308)
        failing = csim.DroneBatch(model, 3, threads=2)
        reference = csim.DroneBatch(model, 3, threads=1)
        randomizer = BatchRandomizer(failing, seed=42)
        repeated = BatchRandomizer(reference, seed=42)
        before = failing.get_state()
        for seed in (None, 71):
            with self.subTest(seed=seed), self.assertRaises(OverflowError):
                randomizer.reset(seed=seed, position_low=[0, 0, 2], position_high=[0, 0, 3])
            for key in before:
                np.testing.assert_array_equal(failing.get_state()[key], before[key])
            self.assertEqual(randomizer.random.metadata["seed"], 42)
        settings = dict(position_low=[0, 0, -.1], position_high=[0, 0, .1])
        randomizer.reset(**settings)
        repeated.reset(**settings)
        np.testing.assert_array_equal(failing.get_state()["state"], reference.get_state()["state"])

    def test_invalid_reset_inputs_and_seed_only_do_not_change_physics(self):
        batch = self.make_batch("drone")
        randomizer = BatchRandomizer(batch, seed=42)
        before = batch.get_state()
        for settings in ({"position_low": [0, 0]}, {"position_high": [0, np.nan, 0]},
                         {"velocity_low": [1, 0, 0]}, {"indices": [1, 1]},
                         {"seed": 17, "indices": [1]}, {"seed": True}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                randomizer.reset(**settings)
        self.assertEqual(randomizer.random.metadata["seed"], 42)
        self.assertEqual(randomizer.seed(np.int64(17)), 17)
        for key in before:
            np.testing.assert_array_equal(batch.get_state()[key], before[key])
        with self.assertRaises(ValueError):
            BatchRandomizer(batch, base_state=np.zeros((2, 13)))
        with self.assertRaises(ValueError):
            BatchRandomizer(csim.DroneModel())


if __name__ == "__main__":
    unittest.main()
