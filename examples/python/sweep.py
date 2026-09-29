"""Scan timestep, payload mass, command delay and steady wind along world X."""
import argparse
import json
from csim_experiments import default_sweep_config, run_sweep
from csim_experiments.recording import read_json

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--config')
    parser.add_argument('--duration',type=float,default=2.)
    parser.add_argument('--timesteps',type=float,nargs='+',default=[.001,.002])
    parser.add_argument('--payload-masses',type=float,nargs='+',default=[.2,.4])
    parser.add_argument('--delays',type=float,nargs='+',default=[0.,.004])
    parser.add_argument('--wind-speeds',type=float,nargs='+',default=[0.,2.])
    args=parser.parse_args()
    config=read_json(args.config) if args.config else default_sweep_config()
    config['experiment']['duration']=args.duration
    print(json.dumps(run_sweep(config,args.output_dir,timesteps=args.timesteps,
        payload_masses=args.payload_masses,delays=args.delays,wind_speeds=args.wind_speeds),indent=2))
