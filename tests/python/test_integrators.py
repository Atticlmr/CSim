import math
from pathlib import Path
import tempfile
import unittest
import csim
from csim_control import ControlConfig, ControlLoop
from csim_experiments import default_config, run_flight, replay, model_from_config

GENERAL=('euler','midpoint','heun','rk4','dopri5')
MECHANICAL=('symplectic_euler','velocity_verlet')


class IntegratorTests(unittest.TestCase):
    def test_freefall_and_exact_outer_clock(self):
        for method in GENERAL:
            model=csim.DroneModel(timestep=.01,integrator=method)
            data=csim.make_data(model,position_W=[0,0,5]); t=0.
            for _ in range(100):
                csim.step(model,data); t+=model.timestep
                self.assertEqual(csim.get_state(model,data)['time'],t)
            state=csim.get_state(model,data)
            self.assertAlmostEqual(state['velocity_W'][2],-model.gravity,places=11)
            expected=5-model.gravity/2+(model.gravity*.01/2 if method=='euler' else 0)
            self.assertAlmostEqual(state['position_W'][2],expected,places=11)
            self.assertEqual(model.integrator,method)

    def test_constant_rotation_and_adaptive_tolerances(self):
        def result(method,dt,tolerance=1e-9,sign=1):
            m=csim.DroneModel(timestep=dt,integrator=method,rtol=tolerance,atol=tolerance*.01)
            d=csim.make_data(m,angular_velocity_B=[0,0,5],q_WB=[sign,0,0,0])
            for _ in range(round(1/dt)): csim.step(m,d)
            q=csim.get_state(m,d)['q_WB']
            self.assertAlmostEqual(sum(x*x for x in q),1,places=13)
            return math.sqrt((q[0]-sign*math.cos(2.5))**2+(q[3]-sign*math.sin(2.5))**2)
        for method in GENERAL:
            self.assertLess(result(method,.02),result(method,.04))
        loose=result('dopri5',1.,1e-3); tight=result('dopri5',1.,1e-9)
        self.assertLess(tight,loose*.01); self.assertLess(tight,1e-8)
        self.assertAlmostEqual(tight,result('dopri5',1.,1e-9,-1),places=14)

    def test_pendulum_methods_and_mechanical_restrictions(self):
        for method in GENERAL+MECHANICAL:
            m=csim.PendulumModel(timestep=.002,integrator=method); d=csim.make_data(m,angle=.2)
            initial=csim.get_state(m,d)['energy']
            for _ in range(1000): csim.step(m,d)
            self.assertLess(abs(csim.get_state(m,d)['energy']-initial)/initial,.05)
        for method in MECHANICAL:
            for cls in (csim.DroneModel,csim.SuspendedPayloadModel):
                with self.assertRaises(ValueError): cls(integrator=method)

    def test_taut_constraints_and_wind_stage_time(self):
        for method in GENERAL:
            m=csim.SuspendedPayloadModel(timestep=.002,integrator=method,
                drone_drag=csim.DragConfig(k1=.1),payload_drag=csim.DragConfig(k1=.02),
                wind=csim.WindField(gust_amplitude_W=[1,0,0],gust_frequency=1.))
            d=csim.make_data(m,thrust=12,cable_direction_W=[.2,0,-math.sqrt(.96)])
            for _ in range(200): csim.step(m,d)
            s=csim.get_state(m,d)
            self.assertAlmostEqual(sum(x*x for x in s['cable_direction_W']),1,places=13)
            self.assertAlmostEqual(sum(a*b for a,b in zip(s['cable_direction_W'],s['cable_angular_velocity_W'])),0,places=13)
            self.assertGreater(s['tension'],0)
            self.assertAlmostEqual(s['aerodynamics']['payload']['wind_velocity_W'][0],math.sin(2*math.pi*s['time']),places=12)
        # Time-dependent wind and velocity-dependent drag: compare to a fine-step reference.
        def run(method,dt):
            m=csim.DroneModel(timestep=dt,integrator=method,rtol=1e-10,atol=1e-12,
                drone_drag=csim.DragConfig(k1=1),wind=csim.WindField(gust_amplitude_W=[2,0,0],gust_frequency=2))
            d=csim.make_data(m)
            for _ in range(round(.2/dt)): csim.step(m,d)
            return csim.get_state(m,d)['position_W']
        for a,b in zip(run('dopri5',.2),run('rk4',.0005)): self.assertAlmostEqual(a,b,places=9)

    def test_adaptive_budget_failure_and_physics_failure_are_atomic(self):
        m=csim.DroneModel(timestep=1.,integrator='dopri5',rtol=1e-12,atol=1e-14,max_substeps=1)
        d=csim.make_data(m,angular_velocity_B=[0,0,20]); before=csim.get_state(m,d)
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError,'max_substeps'): csim.step(m,d)
            self.assertEqual(csim.get_state(m,d),before)
        # Positive tension initially; strong upward gust breaks the taut branch inside the step.
        for method in ('midpoint','heun','rk4','dopri5'):
            m=csim.SuspendedPayloadModel(timestep=.1,integrator=method,payload_drag=csim.DragConfig(k1=1),
                wind=csim.WindField(gust_amplitude_W=[0,0,100],gust_frequency=1))
            d=csim.make_data(m,thrust=12); before=csim.get_state(m,d)
            with self.assertRaises(csim.CableDomainError): csim.step(m,d)
            self.assertEqual(csim.get_state(m,d),before)

    def test_validation_loading_recording_and_python_control_clock(self):
        for kwargs in ({'integrator':'unknown'},{'rtol':-1},{'atol':0},{'atol':math.nan},{'max_substeps':0}):
            with self.assertRaises(ValueError): csim.DroneModel(**kwargs)
        root=Path(__file__).resolve().parents[2]
        m=csim.load_model(str(root/'examples/models/drone.urdf'),free_base=True,integrator='dopri5',rtol=1e-8)
        self.assertEqual(m.integrator,'dopri5'); self.assertEqual(csim.get_config(m)['rtol'],1e-8)
        self.assertEqual(csim.get_config(m)['integrator'],model_from_config(csim.get_config(m)).integrator)
        cfg=default_config('hover'); cfg['model'].update(integrator='dopri5',rtol=1e-8,atol=1e-10)
        cfg['experiment']['duration']=.04
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'run'; run_flight(cfg,output)
            verified=replay(output); self.assertEqual(verified['verified_states'],21)
            self.assertEqual(verified['state']['actuation']['controller_updates'],10)
        # Tight internal integration must not sample a Python controller more often.
        m=csim.DroneModel(timestep=.1,integrator='dopri5',rtol=1e-10,atol=1e-12)
        d=csim.make_data(m,angular_velocity_B=[0,0,20]); loop=ControlLoop(m,d,ControlConfig(mode='ctbr',controller_period=.2))
        loop.set_ctbr(10,[0,0,20]); loop.step(); loop.step()
        self.assertEqual(loop.get_state()['actuation']['controller_updates'],1)
        self.assertEqual(loop.get_state()['time'],.2)
        for name in ('eulerStep','midpointStep','heunStep','DormandPrince54','IntegratorSettings'):
            self.assertFalse(hasattr(csim,name))

if __name__=='__main__': unittest.main()
