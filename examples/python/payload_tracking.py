"""Payload position feedback example; run this file directly from the project root.

Uses an adjustable-speed load reference with a smooth start from rest.
Cable geometry and attitude-rate feedforward compensate faster motion.
The full control/step/read/viewer loop is intentionally visible in this file.
"""
import argparse
from contextlib import ExitStack
import csv
import json
import math
import time

import csim
from csim_control import ControlConfig, ControlLoop
from csim_experiments.flight_control import attitude, cross, dot, multiply


def payload_reference(t, speed=1.0, ramp_time=4.0):
    """World-frame position, velocity, acceleration, jerk and snap.

    speed is the peak reference speed after the startup ramp, not a speed cap
    on the physical payload. For this path, max |v| = 0.625 * phase_rate.
    The phase-rate polynomial has three vanishing derivatives at both ends.
    """
    w = speed / 0.625
    if t < ramp_time:
        u = t / ramp_time
        phase = w * ramp_time * (7*u**5 - 14*u**6 + 10*u**7 - 2.5*u**8)
        d1 = w * (35*u**4 - 84*u**5 + 70*u**6 - 20*u**7)
        d2 = w / ramp_time * (140*u**3 - 420*u**4 + 420*u**5 - 140*u**6)
        d3 = w / ramp_time**2 * (420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5)
        d4 = w / ramp_time**3 * (840*u - 5040*u**2 + 8400*u**3 - 4200*u**4)
    else:
        phase = w * (t - ramp_time / 2)
        d1 = w
        d2 = d3 = d4 = 0.0
    values = [[], [], [], [], []]
    for amplitude, frequency in ((0.5, 1), (0.25, 2)):
        angle = frequency * phase
        a, b, c, d = (frequency*x for x in (d1, d2, d3, d4))
        sn, co = math.sin(angle), math.cos(angle)
        derivatives = (
            amplitude * (1 - co),
            amplitude * sn * a,
            amplitude * (co*a*a + sn*b),
            amplitude * (-sn*a**3 + 3*co*a*b + sn*c),
            amplitude * (-co*a**4 - 6*sn*a*a*b + 3*co*b*b + 4*co*a*c + sn*d),
        )
        for array, value in zip(values, derivatives):
            array.append(value)
    for array, z in zip(values, (4.0, 0.0, 0.0, 0.0, 0.0)):
        array.append(z)
    return values


def cable_reference(acceleration, jerk, snap, gravity):
    """Ideal taut-cable direction and its first two time derivatives.

    With no air forces, the load support force is mL * (aL - gW).
    The drone-to-load direction points opposite to that support force.
    """
    support = list(acceleration)
    support[2] += gravity
    norm = math.sqrt(dot(support, support))
    norm_rate = dot(support, jerk) / norm
    norm_acceleration = (dot(jerk, jerk) + dot(support, snap) - norm_rate**2) / norm
    direction = [-x / norm for x in support]
    rate = [-jerk[i]/norm + support[i]*norm_rate/norm**2 for i in range(3)]
    acceleration = [
        -snap[i]/norm + 2*jerk[i]*norm_rate/norm**2
        + support[i]*norm_acceleration/norm**2 - 2*support[i]*norm_rate**2/norm**3
        for i in range(3)
    ]
    return direction, rate, acceleration


def tracking_command(state, model, reference, previous_attitude, control_period):
    """Return thrust, body rates, desired attitude and command-limit diagnostic.

    Payload position feedback remains outside the drone PD controller.
    Trajectory derivatives come from the planned path, not measured differences.
    The desired attitude rate uses two consecutive commanded attitudes.
    """
    position, velocity, acceleration, jerk, snap = reference
    direction, direction_rate, direction_acceleration = cable_reference(
        acceleration, jerk, snap, model.gravity,
    )
    mass = model.drone_mass + model.payload_mass
    drone_position = [
        position[i] - model.length*direction[i]
        + 0.5*(position[i] - state['payload_position_W'][i])
        for i in range(3)
    ]
    drone_velocity = [velocity[i] - model.length*direction_rate[i] for i in range(3)]
    actual_direction_rate = cross(state['cable_angular_velocity_W'], state['cable_direction_W'])
    # Total thrust supports both masses: mQ*(aQ-gW) + mL*(aL-gW).
    command = [
        acceleration[i] - model.drone_mass/mass*model.length*direction_acceleration[i]
        + 2.0*(drone_position[i] - state['position_W'][i])
        + 2.5*(drone_velocity[i] - state['velocity_W'][i])
        + (0.25*(actual_direction_rate[i] - direction_rate[i]) if i < 2 else 0.0)
        for i in range(3)
    ]
    limits = (6.0, 6.0, 4.0)
    clipped = [max(-bound, min(bound, value)) for bound, value in zip(limits, command)]
    limited = clipped != command
    clipped[2] += model.gravity
    norm = math.sqrt(dot(clipped, clipped))
    desired = attitude([x/norm for x in clipped])
    q = state['q_WB']
    conjugate = [q[0], -q[1], -q[2], -q[3]]
    error = multiply(conjugate, desired)
    sign = 1 if error[0] >= 0 else -1
    rates = [2*10.0*sign*x for x in error[1:]]
    if previous_attitude is not None:
        previous_conjugate = [previous_attitude[0], *[-x for x in previous_attitude[1:]]]
        delta = multiply(desired, previous_conjugate)
        sign = 1 if delta[0] >= 0 else -1
        world_rate = [0.0, *[2*sign*x/control_period for x in delta[1:]]]
        body_rate = multiply(multiply(conjugate, world_rate), q)
        rates = [rates[i] + body_rate[i + 1] for i in range(3)]
    bounded_rates = [max(-3.0, min(3.0, x)) for x in rates]
    limited |= bounded_rates != rates
    w, x, y, z = q
    body_z = [2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y)]
    requested_thrust = mass * dot(clipped, body_z)
    thrust = max(0.1*mass*model.gravity, requested_thrust)
    limited |= thrust != requested_thrust
    return thrust, bounded_rates, desired, limited


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--speed', type=float, default=1.0, help='Peak reference speed after startup, m/s (default: 1.0)')
    parser.add_argument('--ramp-time', type=float, default=4.0, help='Smooth startup duration in seconds (default: 4.0)')
    parser.add_argument('--render-every', type=int, default=10)
    parser.add_argument('--csv', help='Optional new CSV file for payload states and reference')
    args = parser.parse_args()
    dt = 0.002
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error('duration must be finite and in (0, 3600] seconds')
    if not math.isclose(args.duration / dt, round(args.duration / dt), rel_tol=0, abs_tol=1e-8):
        parser.error('duration must be an integer multiple of 0.002 seconds')
    if args.render_every < 1:
        parser.error('render-every must be positive')
    if not math.isfinite(args.speed) or not 0 < args.speed <= 5:
        parser.error('speed must be finite and in (0, 5] m/s; feasibility depends on the controller and trajectory')
    if not math.isfinite(args.ramp_time) or not 0.1 <= args.ramp_time <= 3600:
        parser.error('ramp-time must be finite and in [0.1, 3600] seconds')

    control = ControlConfig(
        mode='ctbr', rate_gain=[16, 16, 8], controller_period=0.004,
        command_delay=0.004, time_constants=[0.02, 0.01, 0.01, 0.01],
        max_thrust=30, max_torque=[1, 1, 0.5], max_rate=[3, 3, 3],
    )
    model = csim.SuspendedPayloadModel(
        drone_mass=1.0, payload_mass=0.2, length=1.0, timestep=dt,
    )
    total_mass = model.drone_mass + model.payload_mass
    p0, *_ = payload_reference(0.0, args.speed, args.ramp_time)
    data = csim.make_data(
        model, position_W=[p0[0], p0[1], p0[2] + model.length],
        cable_direction_W=[0, 0, -1], thrust=total_mass * model.gravity,
    )
    loop = ControlLoop(model, data, control)
    outer_steps = 5  # Python controller: 100 Hz; physics: 500 Hz.
    previous_attitude = None
    state = loop.get_state()
    previous_error_squared = 0.0  # Initial payload position equals the reference.
    error_integral = 0.0
    minimum_tension = state['tension']
    maximum_error = maximum_speed = maximum_reference_speed = maximum_swing = 0.0
    saturated_steps = limited_updates = control_updates = 0
    accepted_steps = 0
    status = 'completed'
    failure = None

    with ExitStack() as stack:
        writer = None
        if args.csv:
            stream = stack.enter_context(open(args.csv, 'x', newline='', encoding='utf-8'))
            writer = csv.writer(stream)
            writer.writerow([
                'time_s', 'reference_x_m', 'reference_y_m', 'reference_z_m',
                'payload_x_m', 'payload_y_m', 'payload_z_m',
                'payload_vx_m_s', 'payload_vy_m_s', 'payload_vz_m_s',
                'payload_ax_m_s2', 'payload_ay_m_s2', 'payload_az_m_s2', 'tension_N',
                'reference_vx_m_s', 'reference_vy_m_s', 'reference_vz_m_s',
                'position_error_m', 'swing_angle_rad', 'applied_thrust_N', 'actuator_saturated',
            ])

        def record(s, desired, desired_velocity):
            if writer is not None:
                writer.writerow([
                    s['time'], *desired, *s['payload_position_W'],
                    *s['payload_velocity_W'], *s['payload_acceleration_W'], s['tension'],
                    *desired_velocity,
                    math.sqrt(sum((a-b)**2 for a, b in zip(s['payload_position_W'], desired))),
                    math.acos(max(-1.0, min(1.0, -s['cable_direction_W'][2]))),
                    s['control']['thrust'], int(s['actuation']['saturated']),
                ])

        record(state, p0, [0, 0, 0])
        viewer = None
        if not args.headless:
            from csim_viewer import Viewer
            viewer = stack.enter_context(Viewer(model))
            viewer.sync(data)
        started = time.monotonic()

        for index in range(round(args.duration / dt)):
            if viewer is not None and not viewer.is_running():
                status = 'closed'
                break
            state = loop.get_state()

            if index % outer_steps == 0:
                reference = payload_reference(index * dt, args.speed, args.ramp_time)
                thrust, rates, previous_attitude, limited = tracking_command(
                    state, model, reference, previous_attitude, outer_steps*dt,
                )
                loop.set_ctbr(thrust, rates)
                control_updates += 1
                limited_updates += int(limited)

            try:
                loop.step()
            except csim.CableDomainError as error:
                status = 'failed'
                failure = str(error)
                break
            state = loop.get_state()
            desired_p, desired_v, *_ = payload_reference((index + 1) * dt, args.speed, args.ramp_time)
            error_squared = sum((a - b)**2 for a, b in zip(state['payload_position_W'], desired_p))
            error_integral += 0.5 * (previous_error_squared + error_squared) * dt
            previous_error_squared = error_squared
            minimum_tension = min(minimum_tension, state['tension'])
            maximum_error = max(maximum_error, math.sqrt(error_squared))
            maximum_speed = max(maximum_speed, math.sqrt(dot(state['payload_velocity_W'], state['payload_velocity_W'])))
            maximum_reference_speed = max(maximum_reference_speed, math.sqrt(dot(desired_v, desired_v)))
            maximum_swing = max(maximum_swing, math.acos(max(-1.0, min(1.0, -state['cable_direction_W'][2]))))
            saturated_steps += int(state['actuation']['saturated'])
            accepted_steps += 1
            record(state, desired_p, desired_v)

            if viewer is not None and (index + 1) % args.render_every == 0:
                viewer.sync(data)
                time.sleep(max(0.0, started + (index + 1) * dt - time.monotonic()))
        if viewer is not None and viewer.is_running():
            viewer.sync(data)

    elapsed = accepted_steps * dt
    print(json.dumps({
        'status': status, 'error': failure, 'duration_s': elapsed, 'accepted_steps': accepted_steps,
        'reference_speed_m_s': args.speed, 'ramp_time_s': args.ramp_time,
        'peak_reference_speed_m_s': maximum_reference_speed,
        'peak_payload_speed_m_s': maximum_speed,
        'payload_position_rmse_m': math.sqrt(error_integral / elapsed) if elapsed else None,
        'payload_position_max_error_m': maximum_error,
        'maximum_swing_deg': math.degrees(maximum_swing),
        'minimum_tension_N': minimum_tension,
        'actuator_saturation_fraction': saturated_steps / accepted_steps if accepted_steps else 0.0,
        'controller_limited_fraction': limited_updates / control_updates if control_updates else 0.0,
        'payload_position_W': state['payload_position_W'],
        'payload_velocity_W': state['payload_velocity_W'],
        'payload_acceleration_W': state['payload_acceleration_W'],
    }, indent=2, allow_nan=False))
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
