"""Record physical poses for the C++ viewer; no graphics dependency in Python.

Example: xmake run csim_python examples/python/record_payload.py build/payload.csv
         xmake run viewer build/payload.csv
"""
import argparse
import csv
import json
import math
import subprocess
from pathlib import Path

import csim


HEADER = ["time", "drone_x", "drone_y", "drone_z", "q_w", "q_x", "q_y", "q_z",
          "payload_x", "payload_y", "payload_z", "tension"]
RIGID_HEADER = HEADER + ["payload_q_w", "payload_q_x", "payload_q_y", "payload_q_z",
    "drone_attachment_x", "drone_attachment_y", "drone_attachment_z",
    "payload_attachment_x", "payload_attachment_y", "payload_attachment_z", "cable_slack", "cable_length"]


def code_version():
    try:
        root = Path(__file__).resolve().parents[2]
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, check=True,
                                  capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=root, check=True,
                                    capture_output=True, text=True, timeout=5).stdout)
        return dict(commit=revision, dirty=dirty)
    except (OSError, subprocess.SubprocessError):
        return dict(commit=None, dirty=None)


def record(path, duration=10.0, timestep=0.002, sample_steps=5, *, model=None, initial=None, thrust=None):
    if model is not None: timestep=model.timestep
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(timestep) or timestep <= 0:
        raise ValueError("Duration and timestep must be finite and positive")
    if not isinstance(sample_steps, int) or sample_steps <= 0:
        raise ValueError("sample_steps must be a positive integer")
    ratio = duration/timestep
    if not math.isfinite(ratio) or ratio > 10_000_000:
        raise ValueError("Recording exceeds ten million physics steps")
    steps = round(ratio)
    if steps < 1 or abs(steps*timestep-duration) > 1e-10*max(1, duration):
        raise ValueError("Duration must be an integer multiple of the physical timestep")
    if steps > 10_000_000 or (steps+sample_steps-1)//sample_steps+1 > 1_000_000:
        raise ValueError("Recording exceeds the step or viewer sample limit")
    path = Path(path)
    metadata_path = Path(str(path)+".json")
    if path.exists() or metadata_path.exists():
        raise FileExistsError("Choose a new output path; existing recordings are not overwritten")
    if model is None:
        model = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2,
                                          gravity=9.81, timestep=timestep)
        direction = [v/math.sqrt(1.13) for v in (0.3, 0.2, -1)]
        rate = [direction[1]*0.1+direction[2]*0.3,
                direction[2]*0.2-direction[0]*0.1, -direction[0]*0.3-direction[1]*0.2]
        if initial is None:
            initial = dict(position_W=[0, 0, 5], cable_direction_W=direction,
                           cable_angular_velocity_W=rate)
    if not isinstance(model,(csim.SuspendedPayloadModel,csim.RigidPayloadModel)):
        raise ValueError("Recording requires a suspended payload model")
    rigid=isinstance(model,csim.RigidPayloadModel)
    if not rigid and model.cable_mode!='taut':
        raise ValueError("Point payload CSV v1 requires strict taut mode")
    initial={} if initial is None else initial
    thrust=(model.drone_mass+model.payload_mass)*model.gravity if thrust is None else thrust
    data = csim.make_data(model, thrust=thrust, **initial)
    path.parent.mkdir(parents=True, exist_ok=True)
    samples = 0
    # Exclusive creation avoids quietly replacing an experiment. Failures leave
    # a valid prefix CSV and no success metadata; the exception is propagated.
    with path.open("x", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(RIGID_HEADER if rigid else HEADER)
        for i in range(steps+1):
            if i:
                csim.step(model, data)
            if i % sample_steps == 0 or i == steps:
                state = csim.get_state(model, data)
                row=[state["time"], *state["position_W"], *state["q_WB"],
                     *state["payload_position_W"], state["tension"]]
                if rigid:
                    row.extend([*state['payload_q_WP'], *state['drone_attachment_position_W'],
                        *state['payload_attachment_position_W'], int(state['mode']=='slack'), model.length])
                writer.writerow(row)
                samples += 1
    model_config=csim.get_config(model)
    drag_enabled=any(model_config[name][key]!=0 for name in ('drone_drag','payload_drag') for key in ('k1','k2','k0'))
    metadata = dict(format="csim-viewer-csv-v2" if rigid else "csim-viewer-csv-v1", completed=True, samples=samples,
                    world_frame="Z-up, local X reference", body_frame="FLU", quaternion="Hamilton wxyz, B to W",
                    units="SI", integrator=model.integrator, timestep=timestep, sample_steps=sample_steps,
                    requested_duration=duration, final_time=state["time"],
                    model=model_config, initial=initial,
                    control=dict(thrust=thrust, torque_B=[0, 0, 0]), damping="aerodynamic" if drag_enabled else "none",
                    code_version=code_version())
    with metadata_path.open("x", encoding="utf-8") as file:
        json.dump(metadata, file, indent=2, allow_nan=False)
        file.write("\n")
    return samples


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--timestep", type=float, default=0.002)
    parser.add_argument("--sample-steps", type=int, default=5)
    parser.add_argument("--drone", type=Path)
    parser.add_argument("--cable", type=Path)
    parser.add_argument("--payload", type=Path)
    args = parser.parse_args()
    paths=(args.drone,args.cable,args.payload)
    if any(paths) and not all(paths): parser.error("Supply --drone, --cable and --payload together")
    model=csim.load_suspended_model(*(str(path) for path in paths),timestep=args.timestep) if all(paths) else None
    count = record(args.output, args.duration, args.timestep, args.sample_steps, model=model)
    print(f"Recorded {count} samples: {args.output} (metadata: {args.output}.json)")
