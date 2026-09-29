import math
import random
import unittest
import csim
from csim_control import ControlConfig, ControlLoop

def channels(w): return [w['thrust'],*w['torque_B']]
A=[[1]*4,[.2,.2,-.2,-.2],[-.2,.2,.2,-.2],[.015,-.015,.015,-.015]]

class AllocationTests(unittest.TestCase):
    def test_bounded_optimum_satisfies_kkt_and_beats_clipping(self):
        upper=[2,3,4,2]; weights=[1,5,5,2]
        m=csim.DroneModel(); config=ControlConfig(allocation_mode='rotor',max_rotor_thrust=upper,allocation_weights=weights)
        rng=random.Random(2718); strictly_better=0
        for _ in range(35):
            w=[rng.uniform(0,16),rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-.2,.2)]
            d=csim.make_data(m); loop=ControlLoop(m,d,config); loop.set_control(w[0],w[1:]); loop.step(); s=loop.get_state()['actuation']
            f=s['target_rotor_thrusts']; residual=channels(s['allocation_residual'])
            self.assertEqual(s['requested_wrench'],{'thrust':w[0],'torque_B':w[1:]})
            for j in range(4):
                self.assertGreaterEqual(f[j],0); self.assertLessEqual(f[j],upper[j])
                gradient=sum(A[i][j]*weights[i]**2*residual[i] for i in range(4))
                if f[j]<1e-9: self.assertGreaterEqual(gradient,-1e-8)
                elif upper[j]-f[j]<1e-9: self.assertLessEqual(gradient,1e-8)
                else: self.assertLess(abs(gradient),1e-8)
            raw=[w[0]/4+w[1]/.8-w[2]/.8+w[3]/.06,w[0]/4+w[1]/.8+w[2]/.8-w[3]/.06,
                 w[0]/4-w[1]/.8+w[2]/.8+w[3]/.06,w[0]/4-w[1]/.8-w[2]/.8-w[3]/.06]
            clipped=[max(0,min(x,u)) for x,u in zip(raw,upper)]
            baseline=sum((weights[i]*(sum(A[i][j]*clipped[j] for j in range(4))-w[i]))**2 for i in range(4))
            optimal=sum((weights[i]*residual[i])**2 for i in range(4))
            self.assertLessEqual(optimal,baseline+1e-10)
            strictly_better+=optimal<baseline-1e-4
        self.assertGreater(strictly_better,10)

    def test_ctbr_delay_allocation_and_per_rotor_response(self):
        m=csim.DroneModel(timestep=.002); config=ControlConfig(mode='ctbr',allocation_mode='rotor',
            controller_period=.004,command_delay=.004,time_constants=[.02]*4,max_rotor_thrust=[5]*4)
        d=csim.make_data(m); loop=ControlLoop(m,d,config); loop.set_ctbr(m.gravity,[0,.2,0])
        for _ in range(2): loop.step()
        self.assertEqual(loop.get_state()['actuation']['rotor_thrusts'],[0]*4)
        loop.step(); s=loop.get_state(); a=s['actuation']
        self.assertAlmostEqual(a['requested_wrench']['torque_B'][1],.02*8*.2)
        for f,target in zip(a['rotor_thrusts'],a['target_rotor_thrusts']):
            self.assertAlmostEqual(f,target*(1-math.exp(-.1)),places=12)
        for error in channels(a['allocation_residual']): self.assertAlmostEqual(error,0,places=12)
        self.assertFalse(a['allocation_saturated'])

    def test_payload_initialization_limits_reset_and_failure_atomicity(self):
        c=ControlConfig(mode='ctbr',allocation_mode='rotor',max_rotor_thrust=[4]*4,time_constants=[.03]*4)
        m=csim.SuspendedPayloadModel(); thrust=(m.drone_mass+m.payload_mass)*m.gravity
        d=csim.make_data(m,thrust=thrust); loop=ControlLoop(m,d,c)
        loop.set_ctbr(thrust,[0,0,0])
        for _ in range(100): loop.step()
        self.assertAlmostEqual(loop.get_state()['position_W'][2],0,places=12)
        before=loop.get_state()
        with self.assertRaises(ValueError): loop.reset(thrust=100)
        self.assertEqual(loop.get_state(),before)
        with self.assertRaises(ValueError): ControlLoop(m,d,ControlConfig(mode='ctbr',allocation_mode='rotor',max_thrust=10))
        loop.reset(thrust=thrust)
        self.assertEqual(loop.get_state()['actuation']['controller_updates'],0)
        for kw in ({'allocation_mode':'oops'},{'allocation_mode':'rotor','allocation_weights':[1,0,1,1]},
                   {'allocation_mode':'rotor','rotor_moment_ratios':[0]*4}):
            with self.assertRaises(ValueError): ControlLoop(m,d,ControlConfig(**kw))

if __name__=='__main__': unittest.main()
