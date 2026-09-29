"""Load a fixed assembly, hover, and read each source link's world-frame state."""
import argparse
import json
from pathlib import Path

import csim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model', nargs='?', default=str(Path(__file__).resolve().parents[1] / 'models/drone.urdf'))
    parser.add_argument('--payload-mass', type=float, default=None, help='Optional separate point payload, kg')
    parser.add_argument('--steps', type=int, default=100)
    args = parser.parse_args()
    if args.steps < 0:
        parser.error('steps must be nonnegative')
    model = csim.load_model(args.model, free_base=True, payload_mass=args.payload_mass, timestep=0.002)
    if args.payload_mass is None:
        thrust = model.mass * model.gravity
        data = csim.make_data(model, position_W=[0, 0, 5])
    else:
        thrust = (model.drone_mass + model.payload_mass) * model.gravity
        data = csim.make_data(model, position_W=[0, 0, 5], thrust=thrust)
    csim.set_control(model, data, thrust)
    for _ in range(args.steps):
        csim.step(model, data)

    # A single lookup: csim.get_link_state(model, data, model.link_names[0]).
    result = {'link_names': model.link_names, 'links': csim.get_link_states(model, data)}
    if args.payload_mass is not None:
        result['payload_position_W'] = csim.get_state(model, data)['payload_position_W']
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
