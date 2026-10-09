"""Optional Python controllers and actuator approximations above the CSim engine.

The native engine never imports this package or runs a feedback controller.
ControlLoop owns scheduling; its step submits one actual wrench to csim.step.
"""
from copy import deepcopy
from dataclasses import dataclass, asdict
import math
import csim
from .allocation import bounded_allocate, multiply, solve


def _vector(value, size, name, *, positive=False, nonnegative=False, infinite=False):
    try: result=tuple(float(x) for x in value)
    except (TypeError,ValueError) as error: raise ValueError(f'Invalid {name}') from error
    if len(result)!=size or any(math.isnan(x) or (not infinite and not math.isfinite(x)) or
                                (positive and x<=0) or (nonnegative and x<0) for x in result):
        raise ValueError(f'Invalid {name}')
    return result


def _ticks(seconds, dt, period=False):
    if period and seconds==0: return 1
    n=seconds/dt
    if not math.isfinite(n) or n<0 or n>100000 or abs(n-round(n))>1e-9 or (period and n<1):
        raise ValueError('Delay/period must be an integer multiple of timestep (at most 100000 ticks)')
    return round(n)


@dataclass(frozen=True, kw_only=True)
class ControlConfig:
    """Python-only parameters; cannot be passed to a native Model constructor."""
    mode: str='wrench'
    allocation_mode: str='direct'
    allocation_weights: tuple=(1.,1.,1.,1.)
    rate_gain: tuple=(8.,8.,4.)
    controller_period: float=0.
    command_delay: float=0.
    time_constants: tuple=(0.,0.,0.,0.)
    gyroscopic_compensation: bool=True
    max_thrust: float=math.inf
    max_torque: tuple=(math.inf,)*3
    max_rate: tuple=(math.inf,)*3
    rotor_positions_B: tuple=((.2,.2,0.),(-.2,.2,0.),(-.2,-.2,0.),(.2,-.2,0.))
    rotor_moment_ratios: tuple=(.015,-.015,.015,-.015)
    max_rotor_thrust: tuple=(math.inf,)*4

    def __post_init__(self):
        if self.mode not in ('wrench','ctbr','rotor_thrust'): raise ValueError('Unknown control mode')
        if self.allocation_mode not in ('direct','rotor'): raise ValueError('Unknown allocation mode')
        for name,n,options in (
            ('allocation_weights',4,dict(positive=True)),('rate_gain',3,dict(nonnegative=True)),
            ('time_constants',4,dict(nonnegative=True)),('max_torque',3,dict(positive=True,infinite=True)),
            ('max_rate',3,dict(positive=True,infinite=True)),('rotor_moment_ratios',4,{}),
            ('max_rotor_thrust',4,dict(positive=True,infinite=True))):
            object.__setattr__(self,name,_vector(getattr(self,name),n,name,**options))
        positions=tuple(_vector(p,3,'rotor_positions_B') for p in self.rotor_positions_B)
        if len(positions)!=4: raise ValueError('Expected four rotor positions')
        object.__setattr__(self,'rotor_positions_B',positions)
        if not self.max_thrust>0: raise ValueError('max_thrust must be positive')
        for value in (self.command_delay,self.controller_period):
            if not math.isfinite(value) or value<0: raise ValueError('Invalid delay/period')
        if self.rotors:
            a=self.matrix(); solve(a,[0.]*4)
            scale=max(self.allocation_weights)
            solve([[x*w/scale for x in row] for row,w in zip(a,self.allocation_weights)],[0.]*4)

    @property
    def rotors(self): return self.mode=='rotor_thrust' or self.allocation_mode=='rotor'

    def matrix(self):
        return [[1.]*4,[p[1] for p in self.rotor_positions_B],[-p[0] for p in self.rotor_positions_B],list(self.rotor_moment_ratios)]

    def to_dict(self):
        def lists(x): return [lists(v) for v in x] if isinstance(x,tuple) else x
        return {key:lists(value) for key,value in asdict(self).items()}


def _channels(w): return [w['thrust'],*w['torque_B']]
def _wrench(v): return {'thrust':v[0],'torque_B':list(v[1:])}
def _cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]


class ControlLoop:
    """Own one Model/Data pair and run the optional Python control chain.

    Commands are queued without mutating physics. step() samples feedback on
    integer ticks, computes exact actuator endpoint states, and holds their
    interval-average wrench constant for one native step. This approximation
    converges with timestep; it is not the old RK-stage actuator coupling.
    On a failed native step, both the old physical input and Python state remain.
    Do not step/reset/write the same Data separately while this loop owns it.
    """
    def __init__(self, model, data, config=None):
        self.model=model; self.data=data; self.config=config or ControlConfig()
        self._period=_ticks(self.config.controller_period,model.timestep,True)
        self._delay=_ticks(self.config.command_delay,model.timestep)
        state=csim.get_state(model,data)
        if not isinstance(model,(csim.DroneModel,csim.SuspendedPayloadModel,csim.RigidPayloadModel)): raise ValueError('Expected a drone model')
        self._matrix=self.config.matrix()
        actual=_channels(state['control']); c=self.config
        if c.mode!='rotor_thrust' and any(abs(x)>u for x,u in zip(actual,[c.max_thrust,*c.max_torque])):
            raise ValueError('Initial actuator wrench exceeds limits')
        if c.rotors:
            actual=solve(self._matrix,actual)
            if any(x < -1e-12 or x>u for x,u in zip(actual,c.max_rotor_thrust)):
                raise ValueError('Initial wrench is infeasible for rotor limits')
            actual=[max(0.,x) for x in actual]
        command=(dict(rotor_thrusts=actual) if c.mode=='rotor_thrust' else
                 dict(thrust=state['control']['thrust'],body_rate_B=state['angular_velocity_B']) if c.mode=='ctbr' else state['control'])
        self._state={'actual':actual,'target':list(actual),'queue':[], 'time':state['time'],
                     'snapshot':{'mode':c.mode,'allocation_mode':c.allocation_mode,'commanded':deepcopy(command),
                        'delayed':deepcopy(command),'target':deepcopy(state['control']),'applied':deepcopy(state['control']),
                        'endpoint':deepcopy(state['control']),'saturated':False,'pending_commands':0,'tick':0,
                        'controller_updates':0}}
        if c.rotors:
            self._state['snapshot'].update(rotor_thrusts=list(actual),target_rotor_thrusts=list(actual),
                requested_wrench=deepcopy(state['control']),allocation_residual=_wrench([0.]*4),allocation_saturated=False,
                rotor_at_limit=[x==0 or x==u for x,u in zip(actual,c.max_rotor_thrust)])

    def _physical(self):
        state=csim.get_state(self.model,self.data)
        if state['time']!=self._state['time'] or state['control']!=self._state['snapshot']['applied']:
            raise ValueError('Data changed outside ControlLoop; recreate the loop after external step/reset/input')
        return state

    def get_state(self):
        """Physical snapshot plus Python-only actuation diagnostics."""
        state=self._physical(); state['actuation']=deepcopy(self._state['snapshot']); return state

    def _submit(self,mode,command):
        self._physical()
        if self.config.mode!=mode: raise ValueError('Command does not match Python control mode')
        candidate=deepcopy(self._state); due=candidate['snapshot']['tick']+self._delay
        if candidate['queue'] and candidate['queue'][-1][0]==due: candidate['queue'][-1]=(due,command)
        else: candidate['queue'].append((due,command))
        candidate['snapshot']['commanded']=deepcopy(command)
        candidate['snapshot']['pending_commands']=len(candidate['queue']); self._state=candidate

    def set_control(self,thrust,torque_B=(0.,0.,0.)):
        values=_vector([thrust,*torque_B],4,'wrench')
        if thrust<0: raise ValueError('Thrust must be nonnegative')
        self._submit('wrench',_wrench(values))

    def set_ctbr(self,thrust,body_rate_B):
        if not math.isfinite(thrust) or thrust<0: raise ValueError('Thrust must be finite and nonnegative')
        self._submit('ctbr',dict(thrust=thrust,body_rate_B=list(_vector(body_rate_B,3,'body_rate_B'))))

    def set_rotor_thrusts(self,thrusts):
        self._submit('rotor_thrust',dict(rotor_thrusts=list(_vector(thrusts,4,'rotor_thrusts',nonnegative=True))))

    def _choose(self,state,rate):
        c=self.config; s=state['snapshot']; command=s['delayed']; s['saturated']=False
        def clip(x,lo,hi):
            value=max(lo,min(x,hi)); s['saturated']|=value!=x; return value
        if c.mode=='rotor_thrust':
            wanted=multiply(self._matrix,command['rotor_thrusts'])
            target=[clip(x,0,u) for x,u in zip(command['rotor_thrusts'],c.max_rotor_thrust)]
        else:
            if c.mode=='ctbr':
                desired=[clip(r,-u,u) for r,u in zip(command['body_rate_B'],c.max_rate)]
                torque=multiply(self.model.inertia_B,[k*(r-v) for k,r,v in zip(c.rate_gain,desired,rate)])
                if c.gyroscopic_compensation:
                    gyro=_cross(rate,multiply(self.model.inertia_B,rate)); torque=[a+b for a,b in zip(torque,gyro)]
                values=[command['thrust'],*torque]
            else: values=_channels(command)
            if not all(math.isfinite(x) for x in values): raise OverflowError('Controller wrench overflow')
            wanted=[clip(values[0],0,c.max_thrust),*[clip(x,-u,u) for x,u in zip(values[1:],c.max_torque)]]
            target=bounded_allocate(self._matrix,wanted,c.allocation_weights,c.max_rotor_thrust) if c.rotors else wanted
        if not all(math.isfinite(x) for x in wanted+target): raise OverflowError('Actuator target overflow')
        state['target']=target; feasible=multiply(self._matrix,target) if c.rotors else target
        s['target']=_wrench(feasible)
        if c.rotors:
            residual=[x-y for x,y in zip(feasible,wanted)]
            saturated=any(abs(x)>1e-10*max(1.,abs(y)) for x,y in zip(residual,wanted))
            s.update(requested_wrench=_wrench(wanted),allocation_residual=_wrench(residual),allocation_saturated=saturated,
                     target_rotor_thrusts=list(target),rotor_at_limit=[x==0 or x==u for x,u in zip(target,c.max_rotor_thrust)])
            s['saturated']|=saturated

    def step(self):
        physical=self._physical(); state=deepcopy(self._state); s=state['snapshot']; c=self.config; dt=self.model.timestep
        while state['queue'] and state['queue'][0][0]<=s['tick']:
            _,s['delayed']=state['queue'].pop(0)
        s['pending_commands']=len(state['queue'])
        if c.mode!='ctbr' or s['tick']%self._period==0:
            self._choose(state,physical['angular_velocity_B'])
            if c.mode=='ctbr': s['controller_updates']+=1
        actual=[]; average=[]
        for start,target,tau in zip(state['actual'],state['target'],c.time_constants):
            if tau==0: end=mean=target
            else:
                ratio=dt/tau; alpha=-math.expm1(-ratio)
                end=start*(1-alpha)+target*alpha
                # Series avoids cancellation for very slow actuators.
                mean_alpha=ratio/2-ratio*ratio/6 if ratio<1e-5 else 1-alpha/ratio
                mean=start*(1-mean_alpha)+target*mean_alpha
            actual.append(end); average.append(mean)
        applied=multiply(self._matrix,average) if c.rotors else average
        endpoint=multiply(self._matrix,actual) if c.rotors else actual
        if not all(math.isfinite(x) for x in actual+applied+endpoint): raise OverflowError('Actuator response overflow')
        s['applied']=_wrench(applied); s['endpoint']=_wrench(endpoint)
        if c.rotors: s['rotor_thrusts']=actual
        state['actual']=actual; s['tick']+=1; state['time']+=dt
        csim.step(self.model,self.data,**s['applied'])
        self._state=state

    def reset(self,**initial):
        """Validate both layers before resetting the owned Data and clearing queues."""
        candidate=csim.make_data(self.model,**initial)
        fresh=ControlLoop(self.model,candidate,self.config)
        csim.reset(self.model,self.data,**initial)
        self._state=fresh._state


__all__=['ControlConfig','ControlLoop']
