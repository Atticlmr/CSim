"""Repeat a randomized reset and trajectory using environment-specific seeds."""

import argparse
import json

import numpy as np

import csim
from csim_experiments import BatchRandomizer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=42)
    options = parser.parse_args()
    model = csim.DroneModel(timestep=.002)
    batch = csim.DroneBatch(model, 8, threads=2)
    randomizer = BatchRandomizer(batch, seed=options.seed)
    settings = dict(position_low=[-1, -1, 1], position_high=[1, 1, 2],
                    velocity_low=[-.1, -.1, -.1], velocity_high=[.1, .1, .1])
    actions = np.tile([model.mass * model.gravity, 0, 0, 0], (batch.num_envs, 1))
    trajectories = []
    for _ in range(2):
        randomizer.reset(seed=options.seed, **settings)
        batch.step(actions, substeps=10)
        trajectories.append(batch.get_state()["state"])
    np.testing.assert_array_equal(*trajectories)
    print(json.dumps({**randomizer.random.metadata, "identical_trajectories": True}, indent=2))


if __name__ == "__main__":
    main()
