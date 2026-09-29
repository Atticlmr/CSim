"""Python owns physics and pacing; the window only displays snapshots."""
import argparse
import time
import math
import csim
from csim_viewer import Viewer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--payload', action='store_true')
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--hidden', action='store_true')
    parser.add_argument('--screenshot')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error("duration must be in (0, 3600] seconds")
    model = csim.SuspendedPayloadModel(timestep=0.002) if args.payload else csim.DroneModel(timestep=0.002)
    thrust = ((model.drone_mass+model.payload_mass) if args.payload else model.mass)*model.gravity
    data = csim.make_data(model, position_W=[0, 0, 5], **({'thrust': thrust} if args.payload else {}))
    csim.set_control(model, data, thrust=thrust)
    with Viewer(model, hidden=args.hidden) as viewer:
        viewer.sync(data)
        start = time.monotonic()
        for index in range(round(args.duration/model.timestep)):
            if not viewer.is_running():
                break
            csim.step(model, data)
            if index % 5 == 4:
                viewer.sync(data)
                if not args.hidden:
                    time.sleep(max(0, start+(index+1)*model.timestep-time.monotonic()))
        if viewer.is_running():
            viewer.sync(data)
            if args.screenshot:
                viewer.screenshot(args.screenshot)


if __name__ == '__main__':
    main()
