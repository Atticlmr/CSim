"""CLI for the installed, reproducible flight experiment runner."""
import argparse
import json
from pathlib import Path
import time
import uuid
import csim
from csim_experiments import default_config, run_flight
from csim_experiments.recording import read_json, write_json


def main(task):
    parser=argparse.ArgumentParser(description=f'CSim {task}: Python control, physics and full recording')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--render-every',type=int,default=10)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--hidden',action='store_true')
    parser.add_argument('--screenshot')
    parser.add_argument('--output',help='Optional additional final JSON file; must not exist')
    parser.add_argument('--output-dir',help='New directory for configuration, commands, states and metrics')
    parser.add_argument('--config',help='JSON with model, control, initial and experiment sections')
    parser.add_argument('--swing-gain',type=float)
    parser.add_argument('--no-swing-damping',action='store_true')
    parser.add_argument('--model',help='URDF or supported MJCF file')
    parser.add_argument('--free-base',action='store_true')
    parser.add_argument('--rotors',action='store_true',help='Enable constrained allocation and per-rotor response')
    args=parser.parse_args()
    config=read_json(args.config) if args.config else default_config(task,rotors=args.rotors)
    if args.config and args.rotors: parser.error('Set allocation_mode in the supplied configuration')
    config['experiment']['task']=task
    if args.duration is not None: config['experiment']['duration']=args.duration
    if args.swing_gain is not None: config['experiment']['swing_gain']=args.swing_gain
    if args.no_swing_damping: config['experiment']['swing_gain']=0.
    model=None
    if args.model:
        cfg=config['model']
        model=csim.load_model(args.model,free_base=args.free_base,timestep=cfg['timestep'],
            gravity=cfg['gravity'],integrator=cfg.get('integrator','rk4'),
            rtol=cfg.get('rtol',1e-6),atol=cfg.get('atol',1e-9),max_substeps=cfg.get('max_substeps',10000),
            drone_drag=csim.DragConfig(**cfg['drone_drag']),wind=csim.WindField(**cfg['wind']),
            payload_mass=cfg.get('payload_mass'),length=cfg.get('length',1.),
            payload_drag=csim.DragConfig(**cfg.get('payload_drag',{})))
        config['model']=csim.get_config(model)
        if isinstance(model,csim.SuspendedPayloadModel): config['initial']['thrust']=(model.drone_mass+model.payload_mass)*model.gravity
    if args.output and Path(args.output).exists(): parser.error('Output file already exists')
    directory=args.output_dir or str(Path('runs')/(time.strftime('%Y%m%d-%H%M%S')+'-'+task+'-'+uuid.uuid4().hex[:8]))
    result=run_flight(config,directory,headless=args.headless,hidden=args.hidden,
        render_every=args.render_every,screenshot=args.screenshot,model=model)
    if args.output: write_json(args.output,result)
    print(json.dumps(result,indent=2,allow_nan=False))
