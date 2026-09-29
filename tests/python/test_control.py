import math
import unittest
import csim
from csim_control import ControlConfig, ControlLoop


class ControlTests(unittest.TestCase):
    def test_native_engine_has_only_actual_inputs(self):
        for name in ('ControlConfig','set_ctbr','set_rotor_thrusts'):
            self.assertFalse(hasattr(csim,name))
        model=csim.DroneModel(); data=csim.make_data(model,angular_velocity_B=[0,0,1])
        self.assertNotIn('control',csim.get_config(model))
        self.assertNotIn('actuation',csim.get_state(model,data))
        with self.assertRaises(TypeError): csim.DroneModel(control=ControlConfig())
        csim.set_control(model,data,100,torque_B=[0,0,2])
        self.assertEqual(csim.get_state(model,data)['control'],{'thrust':100,'torque_B':[0,0,2]})
        csim.step(model,data)
        self.assertAlmostEqual(csim.get_state(model,data)['angular_velocity_B'][2],1+2/model.inertia_B[2][2]*model.timestep)

    def test_actuator_exponential_impulse_and_position_convergence(self):
        errors=[]; tau=.05; thrust=12; duration=.3
        for dt in (.002,.001):
            m=csim.DroneModel(timestep=dt); d=csim.make_data(m)
            loop=ControlLoop(m,d,ControlConfig(time_constants=[tau,0,0,0]))
            loop.set_control(thrust)
            self.assertEqual(csim.get_state(m,d)['control']['thrust'],0)
            for _ in range(round(duration/dt)): loop.step()
            s=loop.get_state(); e=math.exp(-duration/tau)
            self.assertAlmostEqual(s['actuation']['endpoint']['thrust'],thrust*(1-e),places=12)
            velocity=thrust*(duration-tau*(1-e))-m.gravity*duration
            position=thrust*(duration**2/2-tau*duration+tau*tau*(1-e))-m.gravity*duration**2/2
            self.assertAlmostEqual(s['velocity_W'][2],velocity,places=12)
            errors.append(abs(s['position_W'][2]-position))
            self.assertEqual(s['control'],s['actuation']['applied'])
            self.assertLess(s['control']['thrust'],s['actuation']['endpoint']['thrust'])
        self.assertGreater(errors[0]/errors[1],3.9)
        self.assertLess(errors[1],1.1e-6)

    def test_delay_boundary_last_command_and_reset(self):
        m=csim.DroneModel(timestep=.01); d=csim.make_data(m)
        loop=ControlLoop(m,d,ControlConfig(command_delay=.02))
        loop.set_control(8); loop.set_control(12)
        self.assertEqual(loop.get_state()['actuation']['pending_commands'],1)
        for _ in range(2):
            loop.step(); self.assertEqual(loop.get_state()['control']['thrust'],0)
        loop.step(); s=loop.get_state()
        self.assertEqual(s['control']['thrust'],12)
        self.assertAlmostEqual(s['velocity_W'][2],12*.01-m.gravity*.03)
        loop.set_control(15); loop.reset()
        s=loop.get_state()
        self.assertEqual(s['actuation']['pending_commands'],0)
        self.assertEqual(s['actuation']['tick'],0)
        self.assertEqual(s['control']['thrust'],0)

    def test_ctbr_convergence_and_sample_hold(self):
        dt=.002; m=csim.DroneModel(timestep=dt); d=csim.make_data(m)
        loop=ControlLoop(m,d,ControlConfig(mode='ctbr',rate_gain=[8,8,4]))
        loop.set_ctbr(m.mass*m.gravity,[0,0,1])
        for _ in range(500): loop.step()
        s=loop.get_state()
        self.assertAlmostEqual(s['angular_velocity_B'][2],1-(1-4*dt)**500,places=12)
        self.assertEqual(s['actuation']['controller_updates'],500)
        d=csim.make_data(m); loop=ControlLoop(m,d,ControlConfig(mode='ctbr',controller_period=.004))
        loop.set_ctbr(10,[0,0,1]); loop.step(); first=loop.get_state()['control']
        loop.set_ctbr(11,[0,0,-1]); loop.step()
        self.assertEqual(loop.get_state()['control'],first)
        loop.step()
        self.assertLess(loop.get_state()['control']['torque_B'][2],0)
        self.assertEqual(loop.get_state()['control']['thrust'],11)

    def test_ctbr_full_inertia_and_gyroscopic_term(self):
        j=[[2,.1,0],[.1,3,.2],[0,.2,4]]; omega=[.2,-.1,.3]
        m=csim.DroneModel(inertia_B=j,timestep=.001); d=csim.make_data(m,angular_velocity_B=omega)
        loop=ControlLoop(m,d,ControlConfig(mode='ctbr',rate_gain=[0,0,0]))
        loop.set_ctbr(10,[0,0,0]); loop.step()
        momentum=[sum(a*b for a,b in zip(row,omega)) for row in j]
        expected=[omega[1]*momentum[2]-omega[2]*momentum[1],omega[2]*momentum[0]-omega[0]*momentum[2],omega[0]*momentum[1]-omega[1]*momentum[0]]
        for a,b in zip(loop.get_state()['control']['torque_B'],expected): self.assertAlmostEqual(a,b)

    def test_single_rotor_signs_lag_and_saturation(self):
        m=csim.DroneModel()
        for i,expected in enumerate(((.4,-.4,.03),(.4,.4,-.03),(-.4,.4,.03),(-.4,-.4,-.03))):
            d=csim.make_data(m); loop=ControlLoop(m,d,ControlConfig(mode='rotor_thrust'))
            f=[0]*4; f[i]=2; loop.set_rotor_thrusts(f); loop.step(); s=loop.get_state()
            self.assertEqual(s['control']['thrust'],2)
            for a,b in zip(s['control']['torque_B'],expected): self.assertAlmostEqual(a,b)
        d=csim.make_data(m)
        loop=ControlLoop(m,d,ControlConfig(mode='rotor_thrust',time_constants=[.02]*4,max_rotor_thrust=[3]*4))
        loop.set_rotor_thrusts([10]*4)
        for _ in range(20): loop.step()
        s=loop.get_state()
        self.assertTrue(s['actuation']['saturated'])
        self.assertEqual(s['actuation']['target_rotor_thrusts'],[3]*4)
        self.assertAlmostEqual(s['actuation']['endpoint']['thrust'],12*(1-math.exp(-1)),places=12)
        self.assertEqual(s['control']['torque_B'],[0]*3)

    def test_taut_failure_does_not_consume_delayed_command(self):
        m=csim.SuspendedPayloadModel(); hover=(m.drone_mass+m.payload_mass)*m.gravity
        d=csim.make_data(m,thrust=hover); loop=ControlLoop(m,d,ControlConfig(command_delay=.002))
        loop.set_control(0); loop.step(); loop.step(); before=loop.get_state()
        for _ in range(2):
            with self.assertRaises(csim.CableDomainError): loop.step()
            self.assertEqual(loop.get_state(),before)
        loop.reset(thrust=hover); loop.step()
        self.assertEqual(loop.get_state()['actuation']['pending_commands'],0)
        for mode in ('ctbr','rotor_thrust'):
            d=csim.make_data(m,thrust=hover); loop=ControlLoop(m,d,ControlConfig(mode=mode,time_constants=[.05]*4))
            if mode=='ctbr': loop.set_ctbr(hover,[0,0,0])
            else: loop.set_rotor_thrusts([hover/4]*4)
            for _ in range(100): loop.step()
            self.assertAlmostEqual(loop.get_state()['position_W'][2],0,places=12)

    def test_validation_command_atomicity_and_external_mutation(self):
        m=csim.DroneModel(); d=csim.make_data(m)
        for kwargs in ({'command_delay':.0015},{'controller_period':.0005},
                       {'time_constants':[-1,0,0,0]},{'rate_gain':[math.nan,1,1]},
                       {'mode':'rpm'},{'mode':'rotor_thrust','rotor_moment_ratios':[0]*4}):
            with self.assertRaises(ValueError): ControlLoop(m,d,ControlConfig(**kwargs))
        rates=[1]*3; config=ControlConfig(mode='ctbr',max_rate=rates,max_torque=[.01]*3,max_thrust=20)
        rates[0]=100; self.assertEqual(config.max_rate,(1,1,1))
        loop=ControlLoop(m,d,config); loop.set_ctbr(30,[0,0,10]); loop.step(); before=loop.get_state()
        self.assertTrue(before['actuation']['saturated'])
        self.assertEqual(before['control']['thrust'],20)
        self.assertEqual(before['control']['torque_B'][2],.01)
        for action in (lambda:loop.set_control(10),lambda:loop.set_ctbr(10,[0,math.nan,0]),lambda:loop.set_rotor_thrusts([1]*4)):
            with self.assertRaises(ValueError): action()
            self.assertEqual(loop.get_state(),before)
        csim.step(m,d)
        with self.assertRaisesRegex(ValueError,'outside ControlLoop'): loop.step()

    def test_physics_replay_does_not_need_controller(self):
        m=csim.DroneModel(timestep=.002); d=csim.make_data(m); raw=csim.make_data(m)
        loop=ControlLoop(m,d,ControlConfig(mode='ctbr',allocation_mode='rotor',time_constants=[.02]*4,command_delay=.004))
        loop.set_ctbr(12,[.02,-.04,.1])
        for _ in range(100):
            loop.step(); expected=csim.get_state(m,d)
            csim.set_control(m,raw,**expected['control']); csim.step(m,raw)
            self.assertEqual(csim.get_state(m,raw),expected)

if __name__=='__main__': unittest.main()
