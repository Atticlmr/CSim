"""Locate a cable impact inside a physical step and check its analytic solution."""

import math

import csim


def main():
    model = csim.SuspendedPayloadModel(
        drone_mass=1.0,
        payload_mass=0.2,
        length=1.0,
        timestep=0.2,
        cable_mode="hybrid",
        integrator="rk4",
        event_max_step=0.005,
        event_tolerance=1e-10,
    )
    data = csim.make_data(
        model,
        thrust=30.0,
        position_W=[0, 0, 1],
        payload_position_W=[0, 0, 0.1],
        cable_mode="slack",
    )
    initial = csim.get_state(model, data)
    csim.step(model, data)
    state = csim.get_state(model, data)
    expected_time = math.sqrt(2 * (1.0 - 0.9) / 30.0)
    assert len(state["cable_events"]) == 1
    event = state["cable_events"][0]
    assert abs(event["time"] - expected_time) < 2e-10
    assert abs(event["energy_loss"] - 0.5) < 2e-8
    assert state["mode"] == "taut"
    work = 30.0 * (state["position_W"][2] - initial["position_W"][2])
    balance_error = state["energy"] - initial["energy"] - work + event["energy_loss"]
    assert abs(balance_error) < 2e-8
    print(f"Impact time: {event['time']:.10f} s (analytic: {expected_time:.10f} s)")
    print(f"Impulse on drone: {event['impulse_W']} N s")
    print(f"Impact energy loss: {event['energy_loss']:.10f} J")
    print(f"Energy balance error: {balance_error:.3e} J")
    print(f"End of step: {state['time']:.3f} s, mode: {state['mode']}")


if __name__ == "__main__":
    main()
