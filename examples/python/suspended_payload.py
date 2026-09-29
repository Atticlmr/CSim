"""Hover, then an undamped spatial swing with constant collective thrust.

The drone is free to translate. This example has no position/swing controller.
Run: xmake run csim_python examples/python/suspended_payload.py
"""
import math

import csim


model = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2,
                                  gravity=9.81, timestep=0.002)
thrust = (model.drone_mass+model.payload_mass)*model.gravity
data = csim.make_data(model, thrust=thrust, position_W=[0, 0, 5])
for _ in range(500):
    csim.step(model, data)
hover = csim.get_state(model, data)
print(f"Hover: tension={hover['tension']:.6f} N, payload={hover['payload_position_W']}")
if abs(hover["tension"]-model.payload_mass*model.gravity) > 1e-10:
    raise RuntimeError("Hover force balance failed")

scale = math.sqrt(0.3**2+0.2**2+1)
direction = [0.3/scale, 0.2/scale, -1/scale]
# A tangent spatial cable angular velocity, rad/s.
rate = [direction[1]*0.1+direction[2]*0.3,
        direction[2]*0.2-direction[0]*0.1,
        -direction[0]*0.3-direction[1]*0.2]
csim.reset(model, data, thrust=thrust, position_W=[0, 0, 5],
           cable_direction_W=direction, cable_angular_velocity_W=rate)
initial = csim.get_state(model, data)
# Constant world thrust does work on the moving drone; raw energy is not conserved.
invariant = initial["energy"]-thrust*initial["position_W"][2]
max_work_error = 0.0
max_length_error = 0.0
min_tension = initial["tension"]
for _ in range(5000):
    csim.step(model, data)
    state = csim.get_state(model, data)
    max_work_error = max(max_work_error, abs(state["energy"]-thrust*state["position_W"][2]-invariant))
    length = math.sqrt(sum((p-q)**2 for p, q in zip(state["payload_position_W"], state["position_W"])))
    max_length_error = max(max_length_error, abs(length-model.length))
    min_tension = min(min_tension, state["tension"])

print(f"Spatial swing: time={state['time']:.6f} s, min tension={min_tension:.6f} N")
print(f"Drone position: {state['position_W']}")
print(f"Payload position: {state['payload_position_W']}")
print(f"Max |energy - thrust*z - initial|: {max_work_error:.3e} J")
print(f"Max cable length error: {max_length_error:.3e} m")
if max_work_error > 2e-8 or max_length_error > 2e-12 or min_tension <= 0:
    raise RuntimeError("Coupled simulation verification failed")
