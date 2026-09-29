import json
import math
from pathlib import Path
import unittest
import csim

def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]

class AerodynamicsTests(unittest.TestCase):
    def test_disabled_matches_previous_version_bit_for_bit(self):
        expected=json.loads((Path(__file__).resolve().parents[1]/'fixtures/aerodynamics_baseline.json').read_text())
        def subset(actual,old):
            return {k:subset(actual[k],v) for k,v in old.items()} if isinstance(old,dict) else actual
        # The saved v1 controller diagnostics are no longer part of the native API.
        # Keep all original physical baseline values unchanged.
        for state in expected.values(): state.pop('actuation',None)
        for kind in ('drone','payload'):
            for explicit in (False,True):
                kw={'drone_drag':csim.DragConfig(), 'wind':csim.WindField()} if explicit else {}
                if kind=='payload' and explicit: kw['payload_drag']=csim.DragConfig()
                m=(csim.DroneModel if kind=='drone' else csim.SuspendedPayloadModel)(timestep=.002,**kw)
                initial={'position_W':[.1,-.2,5],'angular_velocity_B':[.01,-.02,.03]}
                if kind=='payload': initial.update(thrust=12,cable_direction_W=[.2,0,-math.sqrt(.96)],cable_angular_velocity_W=[0,.1,0])
                d=csim.make_data(m,**initial); csim.set_control(m,d,12,[.001,.002,-.001])
                for _ in range(500): csim.step(m,d)
                self.assertEqual(subset(csim.get_state(m,d),expected[kind]),expected[kind])

    def test_spad_terms_use_absolute_payload_airspeed(self):
        for mode,epsilon in [('exact',0),('tanh',.05)]:
            m=csim.SuspendedPayloadModel(payload_drag=csim.DragConfig(k1=.02,k2=.01,k0=.003,sign_mode=mode,epsilon_v=epsilon),
                wind=csim.WindField(velocity_W=[.3,.1,0]))
            d=csim.make_data(m,thrust=12,velocity_W=[.1,-.2,.3],cable_angular_velocity_W=[0,.1,0])
            s=csim.get_state(m,d); air=s['aerodynamics']['payload']
            v=[a-b for a,b in zip(s['payload_velocity_W'],[.3,.1,0])]
            speed=math.sqrt(dot(v,v))
            for i in range(3):
                self.assertAlmostEqual(air['air_velocity_W'][i],v[i])
                self.assertAlmostEqual(air['linear_force_W'][i],-.02*v[i])
                self.assertAlmostEqual(air['quadratic_force_W'][i],-.005*speed*v[i])
                sign=math.tanh(v[i]/epsilon) if mode=='tanh' else (v[i]>0)-(v[i]<0)
                self.assertAlmostEqual(air['spad_force_W'][i],-.003*sign)
            self.assertLessEqual(air['air_power'],0)
        m=csim.SuspendedPayloadModel(payload_drag=csim.DragConfig(k0=.001))
        d=csim.make_data(m,thrust=12)
        self.assertEqual(csim.get_state(m,d)['aerodynamics']['payload']['spad_force_W'],[0,0,0])

    def test_force_balance_constraint_and_radial_drag(self):
        m=csim.SuspendedPayloadModel(drone_mass=2,payload_mass=.5,length=1.2,
            drone_drag=csim.DragConfig(k1=.15,k2=.02),payload_drag=csim.DragConfig(k1=.03,k0=.002),
            wind=csim.WindField(velocity_W=[1,.3,.2],gradient_W=[[0,0,.2],[0,0,0],[0,0,0]]))
        d=csim.make_data(m,thrust=26,position_W=[0,0,5],velocity_W=[.2,-.1,.4],
            cable_direction_W=[.3,0,-math.sqrt(.91)],cable_angular_velocity_W=[0,.2,0])
        s=csim.get_state(m,d); q=s['cable_direction_W']; dq=cross(s['cable_angular_velocity_W'],q)
        relative=[a-b for a,b in zip(s['payload_acceleration_W'],s['acceleration_W'])]
        self.assertAlmostEqual(dot(q,relative),-m.length*dot(dq,dq),places=12)
        for body,mass,acc,cable,thrust in [('drone',2,s['acceleration_W'],s['cable_force_on_drone_W'],s['thrust_W']),
                                        ('payload',.5,s['payload_acceleration_W'],s['cable_force_on_payload_W'],[0]*3)]:
            air=s['aerodynamics'][body]
            for i in range(3):
                self.assertAlmostEqual(mass*(acc[i]+(m.gravity if i==2 else 0)),
                    air['total_force_W'][i]+cable[i]+thrust[i],places=12)
            z=s['position_W'][2] if body=='drone' else s['payload_position_W'][2]
            self.assertAlmostEqual(air['wind_velocity_W'][0],1+.2*z)

    def test_sinusoidal_wind_is_evaluated_at_rk_stage_times(self):
        gamma=.5; frequency=.7; omega=2*math.pi*frequency; duration=.6
        exact=gamma*2/(gamma*gamma+omega*omega)*(gamma*math.sin(omega*duration)
            -omega*math.cos(omega*duration)+omega*math.exp(-gamma*duration))
        errors=[]
        for dt in (.02,.01):
            m=csim.DroneModel(timestep=dt,drone_drag=csim.DragConfig(k1=gamma),
                wind=csim.WindField(gust_amplitude_W=[2,0,0],gust_frequency=frequency))
            d=csim.make_data(m); csim.set_control(m,d,m.gravity)
            for _ in range(round(duration/dt)): csim.step(m,d)
            s=csim.get_state(m,d)
            errors.append(abs(s['velocity_W'][0]-exact))
            self.assertAlmostEqual(s['aerodynamics']['drone']['wind_velocity_W'][0],2*math.sin(omega*s['time']),places=12)
        self.assertLess(errors[1],errors[0]/14)
        self.assertLess(errors[1],1e-8)

    def test_energy_work_balance_and_wind_power(self):
        dt=.0005
        m=csim.SuspendedPayloadModel(timestep=dt,drone_drag=csim.DragConfig(k1=.1),
            payload_drag=csim.DragConfig(k1=.0034,k0=.001,sign_mode='tanh',epsilon_v=.03),
            wind=csim.WindField(velocity_W=[1,0,0]))
        d=csim.make_data(m,thrust=12,velocity_W=[.1,0,0],position_W=[0,0,5],cable_angular_velocity_W=[0,.2,0])
        def power(s):
            return dot(s['thrust_W'],s['velocity_W'])+dot(s['control']['torque_B'],s['angular_velocity_B'])+sum(a['mechanical_power'] for a in s['aerodynamics'].values())
        old=csim.get_state(m,d); initial=old['energy']; work=0
        self.assertGreater(old['aerodynamics']['drone']['mechanical_power'],0)
        for _ in range(1000):
            csim.step(m,d); new=csim.get_state(m,d); work+=(power(old)+power(new))*dt/2
            for air in new['aerodynamics'].values(): self.assertLessEqual(air['air_power'],1e-14)
            old=new
        self.assertLess(abs(old['energy']-initial-work),1e-6)

    def test_validation_and_wind_failure_atomicity(self):
        for kw in ({'k0':-1},{'k1':math.nan},{'sign_mode':'tanh'},{'epsilon_v':.1},{'sign_mode':'unknown'}):
            with self.assertRaises(ValueError): csim.DragConfig(**kw)
        with self.assertRaises(ValueError): csim.DroneModel(drone_drag=csim.DragConfig(k0=.1))
        with self.assertRaises(ValueError): csim.WindField(gust_frequency=-1)
        m=csim.DroneModel(timestep=10,drone_drag=csim.DragConfig(k1=.1),
            wind=csim.WindField(gust_amplitude_W=[1,0,0],gust_frequency=1e307))
        d=csim.make_data(m); csim.set_control(m,d,m.gravity); before=csim.get_state(m,d)
        with self.assertRaises(OverflowError): csim.step(m,d)
        self.assertEqual(csim.get_state(m,d),before)

if __name__=='__main__': unittest.main()
