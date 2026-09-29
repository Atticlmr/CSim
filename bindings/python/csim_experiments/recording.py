"""Versioned, strict JSON records. No implicit physics or graphics operations."""
import hashlib
from importlib import metadata
import json
import math
from pathlib import Path
import platform
import sys
import csim
import csim_control

SCHEMA='csim-experiment-v2'

def json_safe(value):
    if isinstance(value,float):
        if math.isnan(value): raise ValueError('NaN cannot be recorded')
        if math.isinf(value): return '+inf' if value>0 else '-inf'
    if isinstance(value,dict): return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [json_safe(v) for v in value]
    return value

def restore(value):
    if isinstance(value,str) and value in ('+inf','-inf'): return float(value)
    if isinstance(value,dict): return {k:restore(v) for k,v in value.items()}
    if isinstance(value,list): return [restore(v) for v in value]
    return value

def write_json(path,value):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(json_safe(value),stream,indent=2,allow_nan=False); stream.write('\n')

def read_json(path):
    def invalid(text): raise ValueError(f'Nonstandard JSON number: {text}')
    with Path(path).open(encoding='utf-8') as stream:
        return restore(json.load(stream,parse_constant=invalid))

def provenance():
    def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    try: version=metadata.version('csim-drone')
    except metadata.PackageNotFoundError: version='development'
    return {'csim_version':version,'native_sha256':digest(csim.__file__),
            'python':sys.version,'platform':platform.platform(),
            'control_sources_sha256':{p.name:digest(p) for p in sorted(Path(csim_control.__file__).parent.glob('*.py'))},
            'experiment_sources_sha256':{p.name:digest(p) for p in sorted(Path(__file__).parent.glob('*.py'))}}

class Recording:
    def __init__(self,directory,configuration,initial_state):
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        document={'schema':SCHEMA,**configuration,'initial_state':initial_state,'provenance':provenance()}
        write_json(self.directory/'config.json',document)
        self.states=(self.directory/'states.jsonl').open('x',encoding='utf-8')
        try: self.commands=(self.directory/'commands.jsonl').open('x',encoding='utf-8')
        except BaseException: self.states.close(); raise

    def state(self,step,state,reference):
        self.states.write(json.dumps({'step':step,'state':state,'reference':reference},allow_nan=False)+'\n')

    def command(self,step,time,mode,**values):
        self.commands.write(json.dumps({'step':step,'time':time,'mode':mode,**values},allow_nan=False)+'\n')

    def finish(self,result):
        self.states.close(); self.commands.close()
        write_json(self.directory/'result.json',{'schema':SCHEMA,**result})
