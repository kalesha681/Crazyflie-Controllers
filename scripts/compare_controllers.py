import argparse
import sys
import os
import subprocess

def main():
    parser = argparse.ArgumentParser(description="Legacy Wrapper for Controller Comparison")
    parser.add_argument('--trajectory', type=str, default='eight', choices=['circle', 'square', 'triangle', 'eight', 'step'])
    parser.add_argument('--no-gui', action='store_true')
    
    args = parser.parse_args()
    
    # Map to main.py arguments
    # python scripts/main.py --trajectory {traj} --controllers pid smc mpc --compare --plot [--gui]
    
    cmd = [
        sys.executable,
        os.path.join(os.path.dirname(__file__), 'main.py'),
        '--trajectory', args.trajectory,
        '--controllers', 'pid', 'smc', 'mpc',
        '--plot'
    ]
    
    if not args.no_gui:
        cmd.append('--gui')
        
    print(f"[WRAPPER] Delegating to main.py: {' '.join(cmd)}")
    subprocess.run(cmd)

if __name__ == "__main__":
    main()
