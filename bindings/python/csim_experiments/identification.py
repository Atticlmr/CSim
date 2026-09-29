"""Fit four first-order thrust responses and one shared, tick-aligned delay.

Input is measured thrust in newtons, not PWM or RPM. The gain describes the
steady thrust increment / command increment. This is an offline step experiment.
"""
import csv
import hashlib
import math
from pathlib import Path
import random
import csim
from csim_control import ControlConfig, ControlLoop
from .recording import write_json, provenance

FIELDS=['time',*[f'command_{i}' for i in range(4)],*[f'thrust_{i}' for i in range(4)]]


def synthetic_response(*, timestep=.002, duration=1., step_time=.1, delay=.006,
                       time_constants=(.015,.025,.04,.06), noise_std=0., seed=0):
    if not math.isfinite(noise_std) or noise_std<0: raise ValueError('Invalid noise standard deviation')
    from .flight import ticks
    count=ticks(duration,timestep,'duration'); onset=ticks(step_time,timestep,'step_time')
    if onset>=count: raise ValueError('Step must precede end of experiment')
    config=ControlConfig(mode='rotor_thrust',time_constants=list(time_constants),command_delay=delay)
    model=csim.DroneModel(timestep=timestep); data=csim.make_data(model)
    loop=ControlLoop(model,data,config)
    rng=random.Random(seed); rows=[]
    for index in range(count+1):
        commands=[3.]*4 if index>=onset else [0.]*4
        if index in (0,onset): loop.set_rotor_thrusts(commands)
        state=loop.get_state()
        thrusts=state['actuation']['rotor_thrusts']
        rows.append(dict(zip(FIELDS,[index*timestep,*commands,*[f+rng.gauss(0,noise_std) for f in thrusts]])))
        if index<count: loop.step()
    return rows,{'model':csim.get_config(model),'control':config.to_dict(),'initial_state':csim.get_state(model,csim.make_data(model)),
                 'duration':duration,'step_time':step_time,'noise_std_N':noise_std,'seed':seed,
                 'truth':{'delay_s':delay,'time_constants_s':list(time_constants),'gains':[1.]*4}}


def read_response(path):
    with Path(path).open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream)
        if reader.fieldnames!=FIELDS: raise ValueError('Expected CSV columns '+','.join(FIELDS))
        return [{key:float(value) for key,value in row.items()} for row in reader]


def write_response(path,rows):
    with Path(path).open('x',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=FIELDS); writer.writeheader(); writer.writerows(rows)


def fit_response(rows, *, max_delay=.04):
    if len(rows)<30: raise ValueError('At least 30 samples are required')
    if not math.isfinite(max_delay) or max_delay<0: raise ValueError('Invalid maximum delay')
    if any(set(row)!=set(FIELDS) or any(not math.isfinite(v) for v in row.values()) for row in rows):
        raise ValueError('Response must contain finite values and all CSV fields')
    times=[row['time'] for row in rows]; dt=times[1]-times[0]
    if dt<=0 or any(abs(b-a-dt)>max(1e-10,dt*1e-7) for a,b in zip(times,times[1:])):
        raise ValueError('Samples must have strictly increasing, uniform timestamps')
    if max_delay/dt>2000: raise ValueError('Delay search exceeds 2000 ticks')
    changes=[i for i in range(1,len(rows)) if any(rows[i][f'command_{j}']!=rows[i-1][f'command_{j}'] for j in range(4))]
    if len(changes)!=1: raise ValueError('Expected exactly one simultaneous command step')
    onset=changes[0]; split=onset+int((len(rows)-onset)*.7)
    if onset<5 or split-onset<15 or len(rows)-split<5: raise ValueError('Insufficient baseline, training or validation samples')
    baseline=[sum(row[f'thrust_{j}'] for row in rows[:onset])/onset for j in range(4)]
    increments=[rows[onset][f'command_{j}']-rows[0][f'command_{j}'] for j in range(4)]
    if any(abs(x)<1e-9 for x in increments): raise ValueError('Every rotor must be excited by a nonzero step')
    tau_min=dt*.05; tau_max=times[-1]-times[onset]
    if max_delay>=times[split-1]-times[onset]: raise ValueError('Delay search leaves no training response')
    train=rows[onset:split]; validation=rows[split:]
    def fit_rotor(j,delay):
        ts=[max(0,row['time']-times[onset]-delay) for row in train]
        ys=[row[f'thrust_{j}']-baseline[j] for row in train]
        def evaluate(log_tau):
            tau=math.exp(log_tau); xs=[increments[j]*(-math.expm1(-t/tau)) for t in ts]
            norm=sum(x*x for x in xs)
            gain=sum(x*y for x,y in zip(xs,ys))/norm if norm else 0.
            return sum((y-gain*x)**2 for x,y in zip(xs,ys)),gain,tau
        grid=[math.log(tau_min)+(math.log(tau_max)-math.log(tau_min))*i/40 for i in range(41)]
        best=min(range(len(grid)),key=lambda i:evaluate(grid[i])[0])
        lo=grid[max(0,best-1)]; hi=grid[min(40,best+1)]
        ratio=(math.sqrt(5)-1)/2
        left=hi-ratio*(hi-lo); right=lo+ratio*(hi-lo)
        lv=evaluate(left); rv=evaluate(right)
        for _ in range(45):
            if lv[0]<rv[0]: hi=right; right=left; rv=lv; left=hi-ratio*(hi-lo); lv=evaluate(left)
            else: lo=left; left=right; lv=rv; right=lo+ratio*(hi-lo); rv=evaluate(right)
        return min(lv,rv,key=lambda value:value[0])
    candidates=[]
    for tick in range(math.floor(max_delay/dt+1e-8)+1):
        fitted=[fit_rotor(j,tick*dt) for j in range(4)]
        candidates.append((sum(x[0] for x in fitted),tick*dt,fitted))
    _,delay,fitted=min(candidates,key=lambda item:item[0]); rotors=[]
    for j,(sse,gain,tau) in enumerate(fitted):
        if gain<=0 or tau<=tau_min*1.01 or tau>=tau_max*.99:
            raise ValueError('Response is unidentifiable within the positive first-order model and search bounds')
        # Require a visible rise above baseline noise. Report holdout errors separately.
        noise=math.sqrt(sum((r[f'thrust_{j}']-baseline[j])**2 for r in rows[:onset])/onset)
        if abs(gain*increments[j])<=max(1e-8,5*noise): raise ValueError('Insufficient signal relative to baseline noise')
        def error(row):
            elapsed=max(0,row['time']-times[onset]-delay)
            predicted=baseline[j]+gain*increments[j]*(-math.expm1(-elapsed/tau))
            return (row[f'thrust_{j}']-predicted)**2
        rotors.append({'rotor':j,'time_constant_s':tau,'gain':gain,'baseline_N':baseline[j],
                       'training_rmse_N':math.sqrt(sse/len(train)),
                       'validation_rmse_N':math.sqrt(sum(map(error,validation))/len(validation))})
    return {'model':'first_order_with_shared_tick_delay','delay_s':delay,'sample_period_s':dt,
            'step_time_s':times[onset],'training_samples':len(train),'validation_samples':len(validation),
            'search':{'max_delay_s':max_delay,'tau_bounds_s':[tau_min,tau_max]},'rotors':rotors,
            'delay_at_search_boundary':delay==0 or abs(delay-max_delay)<dt*.5}


def identify(directory, *, input_csv=None, max_delay=.04, noise_std=0., seed=0):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=False)
    if input_csv is None:
        rows,configuration=synthetic_response(noise_std=noise_std,seed=seed)
        write_json(directory/'config.json',{'source':'synthetic CSim experiment','provenance':provenance(),**configuration})
    else:
        rows=read_response(input_csv)
        write_json(directory/'config.json',{'source':str(Path(input_csv).resolve()),
            'source_sha256':hashlib.sha256(Path(input_csv).read_bytes()).hexdigest(),'provenance':provenance()})
    write_response(directory/'response.csv',rows)
    report=fit_response(rows,max_delay=max_delay); write_json(directory/'identification.json',report)
    return report
