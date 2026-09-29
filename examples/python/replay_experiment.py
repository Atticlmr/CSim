"""Verify all recorded physical states from commands and initial conditions."""
import argparse
import json
from csim_experiments import replay

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--tolerance',type=float,default=0.)
    args=parser.parse_args()
    print(json.dumps(replay(args.directory,tolerance=args.tolerance),indent=2))
