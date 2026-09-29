"""Unloaded drone: hover for 1 s, then apply a yaw torque for 1 s."""
import math

import csim


def main():
    model = csim.DroneModel(mass=1, timestep=0.005)
    data = csim.make_data(model, position_W=[0, 0, 5])
    hover_thrust = model.mass * model.gravity
    csim.set_control(model, data, thrust=hover_thrust)
    for _ in range(200):
        csim.step(model, data)
    hover = csim.get_state(model, data)
    print(f"hover: t={hover['time']:.3f} s, position_W={hover['position_W']} m")

    # Open-loop torque, no attitude controller or motor dynamics.
    csim.set_control(model, data, thrust=hover_thrust, torque_B=[0, 0, 0.02])
    for _ in range(200):
        csim.step(model, data)
    state = csim.get_state(model, data)
    expected_rate = 0.02 / model.inertia_B[2][2]
    expected_yaw = 0.5 * expected_rate
    expected_q = [math.cos(expected_yaw/2), 0, 0, math.sin(expected_yaw/2)]
    attitude_error = math.sqrt(sum((a-b)**2 for a, b in zip(state["q_WB"], expected_q)))
    print(f"time: {state['time']:.3f} s")
    print(f"position_W: {state['position_W']} m")
    print(f"angular_velocity_B: {state['angular_velocity_B']} rad/s")
    print(f"q_WB (wxyz): {state['q_WB']}")
    print(f"quaternion error against analytic yaw: {attitude_error:.3e}")
    if attitude_error > 1e-10 or abs(state["angular_velocity_B"][2] - expected_rate) > 1e-11:
        raise RuntimeError("Drone yaw experiment differs from the analytic solution")


if __name__ == "__main__":
    main()
