"""Deterministic flight experiments with full state and command history."""
from copy import deepcopy
from contextlib import nullcontext
import math
import time
import csim
from csim_control import ControlConfig, ControlLoop
from .flight_control import ctbr, reference
from .recording import Recording, restore

def model_from_config(configuration):
    config=restore(deepcopy(configuration)); kind=config.pop('kind')
    config.pop('initial_pose_WB',None)
    if kind not in ('drone','suspended_payload'): raise ValueError('Unsupported experiment model kind')
    config['wind']=csim.WindField(**config.get('wind',{}))
    for key in ('drone_drag','payload_drag'):
        if key in config: config[key]=csim.DragConfig(**config[key])
    return (csim.DroneModel if kind=='drone' else csim.SuspendedPayloadModel)(**config)

def initial_data(model,model_config,initial):
    values=deepcopy(initial)
    for key,value in model_config.get('initial_pose_WB',{}).items():
        if values.get(key) is None: values[key]=value
    return csim.make_data(model,**values)

def default_config(task='tracking',*,rotors=False,payload=None):
    if task not in ('hover','tracking','swing'): raise ValueError('Unknown flight task')
    if payload is None: payload=task=='swing'
    if task=='swing' and not payload: raise ValueError('Swing experiment needs a payload')
    control=ControlConfig(mode='ctbr',rate_gain=[16,16,8],controller_period=.004,
        command_delay=.004,time_constants=[.02]*4 if rotors else [.02,.01,.01,.01],
        max_thrust=30,max_torque=[1,1,.5],max_rate=[3]*3,
        allocation_mode='rotor' if rotors else 'direct',allocation_weights=[1,5,5,2],max_rotor_thrust=[8]*4)
    model=(csim.SuspendedPayloadModel if payload else csim.DroneModel)(timestep=.002)
    initial={'position_W':[0,0,5]}
    if payload:
        initial['thrust']=(model.drone_mass+model.payload_mass)*model.gravity
        initial['cable_direction_W']=[math.sin(.2),0,-math.cos(.2)] if task=='swing' else [0,0,-1]
    return {'model':csim.get_config(model),'control':control.to_dict(),'initial':initial,
            'experiment':{'task':task,'duration':20.,'outer_period':.01,'swing_gain':.25 if task=='swing' else 0.,
                'controller':{'position_gain':2.,'velocity_gain':2.5,'attitude_gain':5.,
                              'acceleration_limits':[3.,3.,4.],'rate_limit':3.,'minimum_thrust_ratio':.1}}}

def ticks(seconds,dt,name):
    n=seconds/dt
    if not math.isfinite(n) or n<1 or abs(n-round(n))>1e-8:
        raise ValueError(f'{name} must be a positive integer multiple of timestep')
    return round(n)

def target(task,t):
    p,v,a=reference(task,t)
    return {'position_W':p,'velocity_W':v,'acceleration_W':a}

class Metrics:
    """Endpoint metrics and trapezoidal integrals, all in explicit SI units."""
    def __init__(self,initial,reference,duration):
        self.duration=duration; self.elapsed=0.; self.samples=0; self.last=initial
        self.error_integral=0.; self.swing_integral=0.; self.thrust_integral=0.; self.torque_integral=[0.]*3
        self.residual_integral=[0.]*4; self.saturated_steps=0; self.last_quarter=[]
        self.tension_min=initial.get('tension'); self.tension_max=initial.get('tension')
        self.previous=self.values(initial,reference)
    @staticmethod
    def values(s,r):
        error=sum((a-b)**2 for a,b in zip(s['position_W'],r['position_W']))
        angle=math.acos(max(-1,min(1,-s['cable_direction_W'][2]))) if 'tension' in s else 0.
        control=s['control']; residual=s['actuation'].get('allocation_residual',{'thrust':0,'torque_B':[0]*3})
        return [error,angle*angle,control['thrust']**2,*[x*x for x in control['torque_B']],
                residual['thrust']**2,*[x*x for x in residual['torque_B']]]
    def add(self,s,r):
        h=s['time']-self.last['time']; values=self.values(s,r)
        integrals=[(a+b)*h/2 for a,b in zip(values,self.previous)]
        self.error_integral+=integrals[0]; self.swing_integral+=integrals[1]; self.thrust_integral+=integrals[2]
        for i in range(3): self.torque_integral[i]+=integrals[3+i]
        for i in range(4): self.residual_integral[i]+=integrals[6+i]
        self.elapsed+=h; self.samples+=1; self.saturated_steps+=s['actuation']['saturated']
        if s['time']>self.duration*.75: self.last_quarter.append(values[:2])
        if 'tension' in s:
            self.tension_min=min(self.tension_min,s['tension']); self.tension_max=max(self.tension_max,s['tension'])
        self.previous=values; self.last=s
    def result(self):
        def rms(value): return math.sqrt(max(0,value)/self.elapsed) if self.elapsed else None
        return {'duration_s':self.elapsed,'accepted_steps':self.samples,'position_rmse_m':rms(self.error_integral),
                'swing_rms_rad':rms(self.swing_integral) if self.tension_min is not None else None,
                'tension_min_N':self.tension_min,'tension_max_N':self.tension_max,
                'thrust_squared_integral_N2_s':self.thrust_integral,
                'torque_squared_integral_N2m2_s':self.torque_integral,
                'allocation_residual_rms':{'thrust_N':rms(self.residual_integral[0]),
                                         'torque_Nm':[rms(x) for x in self.residual_integral[1:]]},
                'saturation_fraction':self.saturated_steps/self.samples if self.samples else 0}

def run_flight(configuration,output_dir,*,headless=True,hidden=False,render_every=10,screenshot=None,model=None):
    config=restore(deepcopy(configuration))
    if set(config)!={'model','control','initial','experiment'}: raise ValueError('Expected model, control, initial and experiment sections')
    options=config['experiment']; allowed={'task','duration','outer_period','swing_gain','controller'}
    if set(options)-allowed: raise ValueError('Unknown experiment options')
    task=options['task']; duration=options['duration']; period=options.get('outer_period',.01); gain=options.get('swing_gain',0.)
    controller=options.get('controller',default_config()['experiment']['controller'])
    defaults=default_config()['experiment']['controller']
    if set(controller)-set(defaults): raise ValueError('Unknown outer controller parameters')
    controller={**defaults,**controller}; options['controller']=controller
    for key,value in controller.items():
        values=value if key=='acceleration_limits' else [value]
        if key=='acceleration_limits' and len(values)!=3: raise ValueError('Expected three acceleration limits')
        if any(not math.isfinite(x) or x<=0 for x in values): raise ValueError('Controller parameters must be finite and positive')
    if controller['acceleration_limits'][2]>=config['model']['gravity']:
        raise ValueError('Vertical acceleration limit must be below gravity for this example controller')
    if task not in ('hover','tracking','swing'): raise ValueError('Unknown task')
    if not math.isfinite(duration) or not 0<duration<=3600: raise ValueError('duration must be in (0,3600] seconds')
    if not math.isfinite(gain) or gain<0: raise ValueError('swing_gain must be finite and nonnegative')
    if not isinstance(render_every,int) or render_every<1: raise ValueError('render_every must be positive integer')
    if screenshot and headless: raise ValueError('screenshot requires a viewer')
    if model is None:
        model=model_from_config(config['model'])
        pose=deepcopy(config['model'].get('initial_pose_WB'))
        config['model']=csim.get_config(model)
        if pose is not None: config['model']['initial_pose_WB']=pose
    else: config['model']=csim.get_config(model)
    control_config=ControlConfig(**config['control'])
    config['control']=control_config.to_dict()
    if control_config.mode!='ctbr': raise ValueError('Flight example requires Python CTBR control')
    payload=isinstance(model,csim.SuspendedPayloadModel)
    if task=='swing' and not payload: raise ValueError('Swing experiment requires a payload')
    count=ticks(duration,model.timestep,'duration'); outer=ticks(period,model.timestep,'outer_period')
    data=initial_data(model,config['model'],config['initial'])
    loop=ControlLoop(model,data,control_config); initial=loop.get_state()
    recorder=Recording(output_dir,config,initial); metrics=Metrics(initial,target(task,0),duration)
    recorder.state(0,initial,target(task,0)); mass=model.drone_mass+model.payload_mass if payload else model.mass
    status='completed'; error=None; stage='viewer'; failure=None
    try:
        if headless: window=nullcontext(None)
        else:
            from csim_viewer import Viewer
            window=Viewer(model,hidden=hidden)
        with window as viewer:
            if viewer: viewer.sync(data)
            start=time.monotonic()
            for index in range(count):
                if viewer and not viewer.is_running(): status='closed'; break
                state=loop.get_state()
                if index%outer==0:
                    stage='control'; desired=target(task,index*model.timestep)
                    thrust,rates=ctbr(state,desired['position_W'],desired['velocity_W'],desired['acceleration_W'],mass,model.gravity,gain if payload else 0.,**controller)
                    loop.set_ctbr(thrust,rates)
                    recorder.command(index,state['time'],'ctbr',thrust=thrust,body_rate_B=rates)
                stage='step'; loop.step()
                state=loop.get_state(); desired=target(task,(index+1)*model.timestep)
                stage='record'; recorder.state(index+1,state,desired); metrics.add(state,desired)
                if viewer and (index+1)%render_every==0:
                    stage='viewer'; viewer.sync(data)
                    if not hidden: time.sleep(max(0,start+(index+1)*model.timestep-time.monotonic()))
            if viewer and viewer.is_running():
                stage='viewer'; viewer.sync(data)
                if screenshot: viewer.screenshot(screenshot)
    except Exception as ex:
        status='failed'; error={'type':type(ex).__name__,'message':str(ex),'stage':stage}; failure=ex
    finally:
        tail=metrics.last_quarter
        result={'status':status,'error':error,'state':loop.get_state(),'metrics':metrics.result(),
                'position_rmse_last_quarter_m':math.sqrt(sum(x[0] for x in tail)/len(tail)) if tail else None,
                'swing_rms_last_quarter_rad':math.sqrt(sum(x[1] for x in tail)/len(tail)) if payload and tail else None,
                'recording_dir':str(recorder.directory)}
        recorder.finish(result)
    if failure is not None: raise failure
    return result
