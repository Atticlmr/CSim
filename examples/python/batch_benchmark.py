import argparse
import json
import os
import platform
from pathlib import Path
from time import perf_counter

import numpy as np

import csim


def positive(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("Require a positive integer")
    return result


def measure(model, batch_type, count, threads, iterations, substeps, read_state):
    batch = batch_type(model, count, threads=threads)
    if hasattr(model, "mass"):
        thrust = model.mass * model.gravity
    else:
        thrust = (model.drone_mass + model.payload_mass) * model.gravity
    actions = np.tile([thrust, 0, 0, .001], (count, 1))
    for _ in range(20):
        batch.step(actions, substeps=substeps)
    start = perf_counter()
    for _ in range(iterations):
        batch.step(actions, substeps=substeps)
        if read_state:
            batch.get_state()
    seconds = perf_counter() - start
    state = batch.get_state()
    if not np.isfinite(state["state"]).all():
        raise RuntimeError("Non-finite benchmark result")
    return dict(batch_type=batch_type.__name__, threads=batch.threads, read_state=read_state, seconds=seconds,
                env_steps_per_second=count * iterations / seconds,
                physical_steps_per_second=count * iterations * substeps / seconds)


def main():
    parser = argparse.ArgumentParser(description="Measure CPU batch throughput for drone and rigid payload environments")
    parser.add_argument("--kind", choices=("drone", "rigid", "both"), default="both")
    parser.add_argument("--num-envs", type=positive, default=1024)
    parser.add_argument("--threads", type=positive, default=4)
    parser.add_argument("--iterations", type=positive, default=200)
    parser.add_argument("--substeps", type=positive, default=4)
    options = parser.parse_args()
    results = []
    configurations = []
    if options.kind in ("drone", "both"):
        configurations.append(("drone", csim.DroneModel(timestep=.002), csim.DroneBatch))
    if options.kind in ("rigid", "both"):
        root = Path(__file__).resolve().parents[1] / "models" / "rigid_payload"
        model = csim.load_suspended_model(
            str(root / "drone.urdf"), str(root / "cable.json"), str(root / "payload.urdf"),
            timestep=.002,
        )
        configurations.append(("rigid_payload", model, csim.RigidPayloadBatch))
    for kind, model, batch_type in configurations:
        for read_state in (False, True):
            serial = measure(model, batch_type, options.num_envs, 1, options.iterations, options.substeps, read_state)
            parallel = measure(model, batch_type, options.num_envs, options.threads, options.iterations, options.substeps, read_state)
            serial["kind"] = parallel["kind"] = kind
            parallel["speedup_vs_one_thread"] = serial["seconds"] / parallel["seconds"]
            results.extend([serial, parallel])
    print(json.dumps(dict(platform=platform.platform(), cpu_count=os.cpu_count(),
                          configuration=vars(options), results=results), indent=2))


if __name__ == "__main__":
    main()
