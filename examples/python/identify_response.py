"""Fit rotor time constants, gains and shared delay from a thrust step response."""
import argparse
import json
from csim_experiments import identify

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--input',help='Measured CSV; omit to generate a synthetic response')
    parser.add_argument('--max-delay',type=float,default=.04)
    parser.add_argument('--noise-std',type=float,default=0.)
    parser.add_argument('--seed',type=int,default=0)
    args=parser.parse_args()
    print(json.dumps(identify(args.output_dir,input_csv=args.input,max_delay=args.max_delay,
        noise_std=args.noise_std,seed=args.seed),indent=2))
