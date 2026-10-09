"""Run a two-body offset cable simulation from three independent files."""
import argparse
from contextlib import nullcontext
from pathlib import Path
import time

import csim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", choices=("urdf", "json", "mjcf"), default="urdf")
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hidden", action="store_true")
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--integrator", default="rk4")
    args = parser.parse_args()
    directory = Path(__file__).resolve().parents[1] / "models" / "rigid_payload"
    extension = "xml" if args.format == "mjcf" else args.format
    model = csim.load_suspended_model(
        str(directory / f"drone.{extension}"),
        str(directory / "cable.json"),
        str(directory / f"payload.{extension}"),
        timestep=0.002, integrator=args.integrator,
    )
    data = csim.make_data(
        model, thrust=(model.drone_mass + model.payload_mass) * model.gravity,
        position_W=[0, 0, 3],
    )
    event_count = 0
    window = nullcontext(None)
    if args.viewer or args.hidden or args.screenshot:
        from csim_viewer import Viewer
        window = Viewer(model, hidden=args.hidden)
    with window as viewer:
        if viewer:
            viewer.sync(data)
        start = time.monotonic()
        for index in range(args.steps):
            if viewer and not viewer.is_running():
                break
            csim.step(model, data)
            event_count += len(csim.get_state(model, data)["cable_events"])
            if viewer and index % 5 == 0:
                viewer.sync(data)
                if not args.hidden:
                    time.sleep(max(0, start + (index + 1)*model.timestep - time.monotonic()))
        if viewer:
            viewer.sync(data)
            if args.screenshot:
                viewer.screenshot(str(args.screenshot))
    state = csim.get_state(model, data)
    print(f"format={args.format}, time={state['time']:.3f}, mode={state['mode']}")
    print("CoM-relative attachments:", model.drone_attachment_B, model.payload_attachment_P)
    print("payload quaternion (wxyz):", state["payload_q_WP"])
    print(f"attachment distance={state['cable_distance']:.9f}, tension={state['tension']:.6f}")
    print("cable events:", event_count)


if __name__ == "__main__":
    main()
