"""Rebuild a recorded experiment from its initial conditions and commands."""
import json
import math
from pathlib import Path
import csim
from csim_control import ControlConfig, ControlLoop
from .flight import initial_data, model_from_config
from .recording import SCHEMA, read_json, provenance


def compare(actual, expected, tolerance, path='state'):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or actual.keys()!=expected.keys():
            raise ValueError(f'{path}: keys differ')
        for key in expected: compare(actual[key], expected[key], tolerance, f'{path}.{key}')
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(actual)!=len(expected): raise ValueError(f'{path}: length differs')
        for index,(a,b) in enumerate(zip(actual,expected)): compare(a,b,tolerance,f'{path}[{index}]')
    elif isinstance(expected,(int,float)) and not isinstance(expected,bool):
        if not isinstance(actual,(int,float)) or not math.isfinite(actual) or not math.isfinite(expected) or abs(actual-expected)>tolerance:
            raise ValueError(f'{path}: {actual!r} != {expected!r}')
    elif actual!=expected: raise ValueError(f'{path}: {actual!r} != {expected!r}')


def rows(path):
    def invalid(value): raise ValueError(f'Invalid JSON number {value}')
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line,parse_constant=invalid)


def replay(directory, *, tolerance=0.):
    """Compare every accepted state. Tolerance is absolute and defaults to exact."""
    if not math.isfinite(tolerance) or tolerance<0: raise ValueError('Invalid replay tolerance')
    directory=Path(directory); config=read_json(directory/'config.json'); result=read_json(directory/'result.json')
    if config.get('schema')!=SCHEMA or result.get('schema')!=SCHEMA: raise ValueError('Unknown recording schema')
    model=model_from_config(config['model']); data=initial_data(model,config['model'],config['initial'])
    loop=ControlLoop(model,data,ControlConfig(**config['control']))
    compare(loop.get_state(),config['initial_state'],tolerance)
    commands=iter(rows(directory/'commands.jsonl')); command=next(commands,None)
    def apply(index):
        nonlocal command
        if command is not None and command['step']<index: raise ValueError('Commands are out of order')
        if command is not None and command['step']==index:
            if command['time']!=loop.get_state()['time']: raise ValueError('Command time differs from physics time')
            if command['mode']!='ctbr': raise ValueError('Unsupported recorded command mode')
            loop.set_ctbr(command['thrust'],command['body_rate_B'])
            command=next(commands,None)
            if command is not None and command['step']<=index: raise ValueError('Duplicate or unsorted command')
    count=0; last_index=None
    for index,row in enumerate(rows(directory/'states.jsonl')):
        if row['step']!=index: raise ValueError('State steps must be consecutive from zero')
        if index:
            apply(index-1); loop.step()
        compare(loop.get_state(),row['state'],tolerance)
        count+=1; last_index=index
    if not count: raise ValueError('Empty state history')
    apply(last_index)
    if command is not None: raise ValueError('Command after final accepted step')
    failure_reproduced=False
    if result['status']=='failed' and result['error']['stage']=='step':
        try: loop.step()
        except Exception as error:
            if type(error).__name__!=result['error']['type']: raise ValueError('Failure type differs') from error
            failure_reproduced=True
        else: raise ValueError('Expected failed physical step succeeded')
    compare(loop.get_state(),result['state'],tolerance)
    if result['metrics']['accepted_steps']!=last_index: raise ValueError('Result step count differs')
    return {'verified_states':count,'accepted_steps':last_index,'failure_reproduced':failure_reproduced,
            'same_native_binary':config['provenance']['native_sha256']==provenance()['native_sha256'],
            'tolerance':tolerance,'state':loop.get_state()}
