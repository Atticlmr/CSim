"""Deterministic Cartesian parameter sweeps with independent recordings."""
from copy import deepcopy
import csv
from itertools import product
import math
from pathlib import Path
from .flight import default_config, run_flight
from .recording import write_json


def default_sweep_config():
    config=default_config('tracking',rotors=True,payload=True)
    # Demonstration coefficients, not identified hardware parameters.
    config['model']['drone_drag']['k1']=.1
    config['model']['payload_drag'].update(k1=.02,k2=.01,k0=.001,sign_mode='tanh',epsilon_v=.02)
    return config


def run_sweep(configuration, directory, *, timesteps, payload_masses, delays, wind_speeds):
    axes={'timestep':list(timesteps),'payload_mass':list(payload_masses),
          'command_delay':list(delays),'wind_speed':list(wind_speeds)}
    if configuration['model']['kind']!='suspended_payload': raise ValueError('Mass sweep requires a payload model')
    for name,values in axes.items():
        if not values or any(not math.isfinite(x) or (x<=0 if name in ('timestep','payload_mass') else x<0) for x in values):
            raise ValueError(f'Invalid sweep axis {name}')
    if math.prod(map(len,axes.values()))>10000: raise ValueError('Sweep exceeds 10000 runs')
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=False)
    write_json(directory/'sweep.json',{'base_configuration':configuration,'axes':axes,
        'initial_thrust_policy':'reset to total weight after changing payload mass','wind_direction_W':[1,0,0]})
    summaries=[]
    with (directory/'summary.csv').open('x',newline='',encoding='utf-8') as stream:
        fields=['run','status',*axes,'position_rmse_m','swing_rms_rad','tension_min_N','tension_max_N',
                'thrust_squared_integral_N2_s','saturation_fraction','error']
        writer=csv.DictWriter(stream,fieldnames=fields); writer.writeheader()
        for index,values in enumerate(product(*axes.values())):
            params=dict(zip(axes,values)); config=deepcopy(configuration); model=config['model']
            model['timestep']=params['timestep']; model['payload_mass']=params['payload_mass']
            config['control']['command_delay']=params['command_delay']; model['wind']['velocity_W']=[params['wind_speed'],0,0]
            config['initial']['thrust']=(model['drone_mass']+model['payload_mass'])*model['gravity']
            name=f'run_{index:04d}'; row={'run':name,**params}
            try:
                result=run_flight(config,directory/name)
                row.update(status=result['status'],**{key:result['metrics'][key] for key in fields if key in result['metrics']})
            except Exception as error:
                row.update(status='failed',error=f'{type(error).__name__}: {error}')
                # Constructor failures occur before a full Recording exists.
                if not (directory/name).exists():
                    (directory/name).mkdir()
                    write_json(directory/name/'rejected_config.json',config)
                    write_json(directory/name/'error.json',{'error':row['error']})
            summaries.append(row); writer.writerow(row); stream.flush()
    write_json(directory/'summary.json',{'runs':summaries})
    return summaries
