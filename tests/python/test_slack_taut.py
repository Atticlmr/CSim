import math
import unittest

import csim


class SlackTautCableTests(unittest.TestCase):
    def test_hybrid_slack_is_independent_until_recontact(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.01)
        data = csim.make_data(
            model,
            thrust=0.0,
            position_W=[0, 0, 5],
            payload_position_W=[0, 0, 4.5],
            cable_mode="slack",
        )
        for _ in range(20):
            csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertEqual(state["mode"], "slack")
        self.assertAlmostEqual(state["tension"], 0.0)
        self.assertAlmostEqual(state["cable_distance"], 0.5, places=10)

        # A slack cable must not change a freely rotating drone's trajectory,
        # including the quaternion treatment in intermediate ODE stages.
        for method in ("euler", "midpoint", "heun", "rk4", "dopri5"):
            with self.subTest(integrator=method):
                coupled = csim.SuspendedPayloadModel(
                    cable_mode="hybrid", timestep=0.005, integrator=method)
                drone = csim.DroneModel(timestep=0.005, integrator=method)
                pose = dict(position_W=[0, 0, 5], q_WB=[0.98, 0.2, 0, 0],
                            angular_velocity_B=[3, -2, 1])
                cdata = csim.make_data(coupled, thrust=0, **pose,
                    payload_position_W=[0, 0, 4.5], cable_mode="slack")
                ddata = csim.make_data(drone, **pose)
                for model, data in ((coupled, cdata), (drone, ddata)):
                    csim.set_control(model, data, thrust=0, torque_B=[0, 0.003, 0.005])
                for _ in range(20):
                    csim.step(coupled, cdata)
                    csim.step(drone, ddata)
                a, b = csim.get_state(coupled, cdata), csim.get_state(drone, ddata)
                for key in ("position_W", "velocity_W", "q_WB", "angular_velocity_B"):
                    self.assertLess(math.dist(a[key], b[key]), 1e-10)

    def test_hybrid_releases_when_tension_becomes_nonpositive(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.01)
        data = csim.make_data(model, thrust=12.0, position_W=[0, 0, 5])
        self.assertGreater(csim.get_state(model, data)["tension"], 0.0)
        csim.set_control(model, data, thrust=0.0)
        csim.step(model, data)
        state = csim.get_state(model, data)
        self.assertEqual(state["mode"], "slack")
        self.assertAlmostEqual(state["tension"], 0.0)

    def test_hybrid_recontact_projects_and_reports_impulse(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.01)
        data = csim.make_data(
            model,
            thrust=30.0,
            position_W=[0, 0, 1],
            payload_position_W=[0, 0, 0.1],
            cable_mode="slack",
        )
        for _ in range(100):
            csim.step(model, data)
            if csim.get_state(model, data)["mode"] == "taut":
                break
        state = csim.get_state(model, data)
        self.assertEqual(state["mode"], "taut")
        self.assertAlmostEqual(state["cable_distance"], model.length, places=10)
        self.assertAlmostEqual(state["cable_radial_velocity"], 0.0, places=10)
        self.assertGreater(sum(x * x for x in state["cable_impulse_W"]), 0.0)

    def test_control_release_survives_step_once_and_reset_clears_it(self):
        for method in ("euler", "rk4", "dopri5", "bdf2", "lie_rk4", "rattle"):
            for atomic in (False, True):
                with self.subTest(method=method, atomic=atomic):
                    model = csim.SuspendedPayloadModel(cable_mode="hybrid", integrator=method)
                    hover = (model.drone_mass + model.payload_mass) * model.gravity
                    data = csim.make_data(model, thrust=hover)
                    csim.step(model, data)
                    release_time = csim.get_state(model, data)["time"]
                    if atomic:
                        csim.step(model, data, thrust=0, torque_B=[0, 0, 0])
                    else:
                        csim.set_control(model, data, thrust=0)
                        csim.set_control(model, data, thrust=0)
                        self.assertEqual(len(csim.get_state(model, data)["cable_events"]), 1)
                        csim.step(model, data)
                    state = csim.get_state(model, data)
                    self.assertEqual(state["mode"], "slack")
                    self.assertEqual(len(state["cable_events"]), 1)
                    event = state["cable_events"][0]
                    self.assertEqual(event["type"], "release")
                    self.assertEqual(event["time"], release_time)
                    self.assertEqual(event["impulse_W"], [0, 0, 0])
                    self.assertEqual(event["energy_loss"], 0)
                    csim.step(model, data)
                    self.assertEqual(csim.get_state(model, data)["cable_events"], [])
                    csim.reset(model, data, thrust=hover)
                    csim.set_control(model, data, thrust=0)
                    csim.reset(model, data, thrust=hover)
                    csim.step(model, data)
                    self.assertEqual(csim.get_state(model, data)["cable_events"], [])

    def test_pending_release_order_event_budget_and_retry(self):
        for limit in (1, 2):
            with self.subTest(limit=limit):
                model = csim.SuspendedPayloadModel(cable_mode="hybrid", max_events=limit)
                data = csim.make_data(model, thrust=12)
                csim.set_control(model, data, thrust=0)
                csim.set_control(model, data, thrust=12)
                before = csim.get_state(model, data)
                if limit == 1:
                    with self.assertRaisesRegex(RuntimeError, "event limit"):
                        csim.step(model, data)
                    self.assertEqual(csim.get_state(model, data), before)
                    csim.set_control(model, data, thrust=0)
                    csim.step(model, data)
                    self.assertEqual([event["type"] for event in csim.get_state(model, data)["cable_events"]], ["release"])
                else:
                    csim.step(model, data)
                    events = csim.get_state(model, data)["cable_events"]
                    self.assertEqual([event["type"] for event in events], ["release", "impact"])
                    self.assertEqual([event["time"] for event in events], [0, 0])
                csim.step(model, data)
                self.assertEqual(csim.get_state(model, data)["cable_events"], [])

    def test_failed_release_step_preserves_pending_events_and_control(self):
        model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=.02, max_substeps=1)
        data = csim.make_data(model, thrust=12)
        before = csim.get_state(model, data)
        with self.assertRaises(RuntimeError):
            csim.step(model, data, thrust=0, torque_B=[0, 0, 0])
        self.assertEqual(csim.get_state(model, data), before)
        csim.set_control(model, data, thrust=0)
        before = csim.get_state(model, data)
        with self.assertRaises(RuntimeError):
            csim.step(model, data)
        self.assertEqual(csim.get_state(model, data), before)

    def test_default_taut_mode_keeps_strict_domain_check(self):
        model = csim.SuspendedPayloadModel(timestep=0.01)
        data = csim.make_data(model, thrust=model.gravity * (model.drone_mass + model.payload_mass), position_W=[0, 0, 5])
        with self.assertRaises(csim.CableDomainError):
            csim.set_control(model, data, thrust=0.0)

    def test_analytic_impact_time_remaining_motion_and_energy(self):
        # Vertical separation d=.9+.5*30*t^2; after impact both bodies share
        # acceleration 30/1.2-g. This tests the remainder of the SAME step.
        expected_time=math.sqrt(.2/30)
        for method in ("midpoint", "heun", "rk4", "dopri5"):
            for dt in (.2, .05):
                with self.subTest(method=method, dt=dt):
                    m=csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=dt,
                        integrator=method, event_max_step=.05, event_tolerance=1e-11)
                    d=csim.make_data(m, thrust=30, position_W=[0,0,1],
                        payload_position_W=[0,0,.1], cable_mode="slack")
                    before=csim.get_state(m,d); events=[]
                    for _ in range(round(.2/dt)):
                        csim.step(m,d); events+=csim.get_state(m,d)["cable_events"]
                    end=csim.get_state(m,d)
                    self.assertEqual(len(events),1)
                    event=events[0]
                    self.assertEqual(event["type"],"impact")
                    self.assertAlmostEqual(event["time"],expected_time,delta=2e-10)
                    self.assertAlmostEqual(event["energy_loss"],.5,delta=2e-9)
                    self.assertAlmostEqual(event["impulse_W"][2],-30*expected_time/6,delta=2e-9)
                    self.assertEqual(end["mode"],"taut")
                    velocity=(25-m.gravity)*.2
                    self.assertAlmostEqual(end["velocity_W"][2],velocity,delta=2e-9)
                    self.assertAlmostEqual(end["payload_velocity_W"][2],velocity,delta=2e-9)
                    com0=(1+.2*.1)/1.2
                    com=com0+.5*(25-m.gravity)*.2**2
                    self.assertAlmostEqual(end["position_W"][2],com+1/6,delta=2e-9)
                    # Total external momentum and mechanical energy minus thrust work.
                    self.assertAlmostEqual(end["velocity_W"][2]+.2*end["payload_velocity_W"][2],
                        (30-1.2*m.gravity)*.2,delta=2e-9)
                    work=30*(end["position_W"][2]-before["position_W"][2])
                    self.assertAlmostEqual(before["energy"]+work-end["energy"],.5,delta=2e-8)

    def test_impulse_can_be_followed_by_slack_and_preserves_momentum(self):
        # Payload ABOVE the drone moving outwards. Upward thrust subsequently
        # closes the gap: an impact occurs but positive tension is impossible.
        m=csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=.15,event_max_step=.01)
        d=csim.make_data(m,thrust=1,position_W=[0,0,0],velocity_W=[.3,.2,0],
            payload_position_W=[0,0,.9],payload_velocity_W=[.3,.2,1],cable_mode="slack")
        csim.step(m,d); s=csim.get_state(m,d)
        self.assertEqual(s["mode"],"slack")
        self.assertLess(s["cable_distance"],1)
        e=s["cable_events"][0]
        self.assertEqual(e["mode_after"],"slack")
        self.assertAlmostEqual(e["time"],1-math.sqrt(.8),delta=2e-9)
        self.assertGreater(e["impulse_W"][2],0)
        self.assertAlmostEqual(s["velocity_W"][2]+.2*s["payload_velocity_W"][2],
            .2+(1-1.2*m.gravity)*.15,delta=2e-9)

    def test_inward_boundary_coincidence_and_neutral_freefall(self):
        for distance, speed in ((1,-1),(.0,0),(1,0)):
            m=csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=.02)
            d=csim.make_data(m,thrust=0,position_W=[0,0,1],
                payload_position_W=[0,0,1-distance],payload_velocity_W=[0,0,-speed],cable_mode="slack")
            for _ in range(20):
                csim.step(m,d); s=csim.get_state(m,d)
                self.assertEqual(s["mode"],"slack")
                self.assertEqual(s["cable_events"],[])
                self.assertEqual(s["cable_impulse_W"],[0,0,0])
            self.assertAlmostEqual(s["cable_distance"],distance+speed*.4,delta=1e-12)

    def test_initial_and_control_release_never_exert_compression(self):
        m=csim.SuspendedPayloadModel(cable_mode="hybrid")
        d=csim.make_data(m,thrust=12,cable_direction_W=[0,0,1])
        self.assertEqual(csim.get_state(m,d)["mode"],"slack")
        csim.reset(m,d,thrust=12)
        csim.set_control(m,d,thrust=0)
        s=csim.get_state(m,d)
        self.assertEqual(s["mode"],"slack")
        self.assertEqual(s["tension"],0)
        self.assertEqual(s["cable_events"][0]["type"],"release")
        self.assertEqual(s["cable_events"][0]["time"],0)
        csim.step(m,d)
        self.assertEqual(csim.get_state(m,d)["cable_events"],s["cable_events"])
        csim.step(m,d)
        self.assertEqual(csim.get_state(m,d)["cable_events"],[])

    def test_budget_failure_and_invalid_control_are_atomic(self):
        # A root-search budget failure must not change the public state.
        m=csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=.01,
            event_max_step=.005,max_substeps=3)
        d=csim.make_data(m,thrust=12,position_W=[0,0,1],
            payload_position_W=[0,0,0],payload_velocity_W=[0,0,-1],cable_mode="slack")
        csim.step(m,d)
        s=csim.get_state(m,d)
        self.assertGreater(s["impact_energy_loss"],0)
        with self.assertRaises(ValueError): csim.set_control(m,d,thrust=-1)
        self.assertEqual(csim.get_state(m,d),s)
        # Reset a separate state near an impact that requires root iterations.
        csim.reset(m,d,thrust=30,position_W=[0,0,1],
            payload_position_W=[0,0,.0001],cable_mode="slack")
        before=csim.get_state(m,d)
        with self.assertRaises(RuntimeError): csim.step(m,d)
        self.assertEqual(csim.get_state(m,d),before)

    def test_release_time_and_trajectory_converge_across_integrators(self):
        def run(method, step):
            m=csim.SuspendedPayloadModel(drone_mass=2,payload_mass=.5,length=1.2,
                gravity=9.81,cable_mode="hybrid",timestep=.2,event_max_step=step,
                event_tolerance=1e-12,integrator=method,rtol=1e-11,atol=1e-13)
            d=csim.make_data(m,thrust=24.525,
                cable_direction_W=[math.sin(1.55),0,-math.cos(1.55)],
                cable_angular_velocity_W=[0,-1,0])
            csim.step(m,d)
            s=csim.get_state(m,d)
            self.assertEqual(s["mode"],"slack")
            self.assertEqual(len(s["cable_events"]),1)
            self.assertEqual(s["cable_events"][0]["type"],"release")
            return s
        reference=run("dopri5",.001)
        t=reference["cable_events"][0]["time"]
        self.assertTrue(0<t<.2)
        for method in ("midpoint","heun","rk4"):
            coarse,fine=run(method,.02),run(method,.01)
            error=lambda s: abs(s["cable_events"][0]["time"]-t)
            self.assertLess(error(fine),error(coarse)*.65)
            position_error=lambda s: math.dist(s["payload_position_W"],reference["payload_position_W"])
            self.assertLess(position_error(fine),position_error(coarse)*.7)

    def test_multiple_events_and_atomic_failure_after_an_impact(self):
        def model(dt,limit=64):
            return csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=dt,
                event_max_step=.005,max_events=limit,payload_drag=csim.DragConfig(k1=.2),
                wind=csim.WindField(gust_amplitude_W=[0,0,30],gust_frequency=1))
        m=model(1); d=csim.make_data(m,thrust=1,position_W=[0,0,2])
        csim.step(m,d); combined=csim.get_state(m,d)
        events=combined["cable_events"]
        self.assertEqual([e["type"] for e in events],["release","impact","release"])
        self.assertEqual([e["time"] for e in events],sorted(e["time"] for e in events))
        self.assertEqual(combined["impact_energy_loss"],sum(e["energy_loss"] for e in events))
        self.assertEqual(combined["cable_impulse_W"],events[1]["impulse_W"])
        # Equivalent fine external clock; events do not call Python controllers.
        small=model(.01); data=csim.make_data(small,thrust=1,position_W=[0,0,2])
        collected=[]
        for _ in range(100):
            csim.step(small,data); collected+=csim.get_state(small,data)["cable_events"]
        self.assertEqual(len(collected),3)
        for a,b in zip(events,collected): self.assertAlmostEqual(a["time"],b["time"],delta=2e-8)
        self.assertLess(math.dist(combined["payload_position_W"],csim.get_state(small,data)["payload_position_W"]),1e-7)
        # Third event exceeds budget AFTER the impact. Nothing gets committed.
        limited=model(1,2); data=csim.make_data(limited,thrust=1,position_W=[0,0,2])
        before=csim.get_state(limited,data)
        with self.assertRaisesRegex(RuntimeError,"event limit"):
            csim.step(limited,data)
        self.assertEqual(csim.get_state(limited,data),before)
        # Existing nonzero impulse diagnostics survive a later failed step too.
        m=csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=.01,event_max_step=.005,max_substeps=3)
        data=csim.make_data(m,thrust=12,position_W=[0,0,1],
            payload_position_W=[0,0,0],payload_velocity_W=[0,0,-1],cable_mode="slack")
        csim.step(m,data)
        csim.set_control(m,data,thrust=12,torque_B=[0,1e7,0])
        before=csim.get_state(m,data)
        self.assertGreater(before["impact_energy_loss"],0)
        with self.assertRaises((RuntimeError,ValueError,OverflowError)): csim.step(m,data)
        self.assertEqual(csim.get_state(m,data),before)

    def test_impact_exactly_at_end_and_snapshot_lifetime(self):
        m=csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=.1,event_max_step=.01)
        d=csim.make_data(m,thrust=0,position_W=[0,0,1],
            payload_position_W=[0,0,.1],payload_velocity_W=[0,0,-1],cable_mode="slack")
        csim.step(m,d); old=csim.get_state(m,d)
        self.assertEqual(len(old["cable_events"]),1)
        self.assertAlmostEqual(old["cable_events"][0]["time"],.1,delta=1e-9)
        self.assertAlmostEqual(old["impact_energy_loss"],1/12,delta=1e-12)
        csim.step(m,d)
        self.assertEqual(csim.get_state(m,d)["cable_events"],[])
        self.assertEqual(len(old["cable_events"]),1)
        old["cable_events"][0]["impulse_W"][0]=999
        self.assertEqual(csim.get_state(m,d)["cable_impulse_W"],[0,0,0])

    def test_euler_impact_converges_and_tangent_release_fails_atomically(self):
        errors=[]
        for h in (.01,.005):
            m=csim.SuspendedPayloadModel(cable_mode="hybrid",timestep=.2,
                integrator="euler",event_max_step=h)
            d=csim.make_data(m,thrust=30,position_W=[0,0,1],
                payload_position_W=[0,0,.1],cable_mode="slack")
            csim.step(m,d)
            errors.append(abs(csim.get_state(m,d)["cable_events"][0]["time"]-math.sqrt(.2/30)))
        self.assertLess(errors[1],errors[0]*.6)
        # Explicit Euler advances a tangent position along a straight line,
        # outside the sphere even when the true solution moves inward. Fail
        # instead of inventing a negative cable impulse or silently using RK4.
        m=csim.SuspendedPayloadModel(drone_mass=2,payload_mass=.5,length=1.2,
            gravity=9.81,cable_mode="hybrid",timestep=.2,integrator="euler")
        d=csim.make_data(m,thrust=24.525,cable_direction_W=[math.sin(1.55),0,-math.cos(1.55)],
            cable_angular_velocity_W=[0,-1,0])
        before=csim.get_state(m,d)
        with self.assertRaisesRegex(RuntimeError,"not outward"): csim.step(m,d)
        self.assertEqual(csim.get_state(m,d),before)

    def test_configuration_roundtrip_and_validation(self):
        from csim_experiments.flight import model_from_config
        m=csim.SuspendedPayloadModel(cable_mode="hybrid",event_max_step=.003,event_tolerance=1e-9,max_events=8)
        self.assertEqual(csim.get_config(model_from_config(csim.get_config(m))),csim.get_config(m))
        for options in ({"event_max_step":0},{"event_tolerance":0},{"max_events":0},
                        {"event_tolerance":.1},{"event_max_step":math.inf}):
            with self.assertRaises(ValueError): csim.SuspendedPayloadModel(**options)
        loaded=csim.load_model("examples/models/drone.urdf",free_base=True,payload_mass=.2,
            cable_mode="hybrid",event_max_step=.003,max_events=8)
        self.assertEqual(loaded.event_max_step,.003)
        self.assertEqual(loaded.max_events,8)


if __name__ == "__main__":
    unittest.main()
