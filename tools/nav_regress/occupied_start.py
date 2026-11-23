#!/usr/bin/env python3
"""Real Nav2 acceptance for starts already overlapping a static obstacle.

The plant models kinematics and false map overlap, not real penetration/contact physics.
Production observation/trust gates and collision monitor stay enabled.
"""
import argparse
import math
import os
import traceback
from pathlib import Path
import numpy as np
import rclpy
from support.plant import Plant
from support.runtime import finish, navigation
from support.terrain import fixture

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--recorded-exit', action='store_true', help='use the real map and the two recorded exit poses')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '93')
    out = Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    terrain = Path(__file__).resolve().parents[2]/'maps/sim/terrain' if args.recorded_exit else fixture(out)
    rclpy.init(); plant = Plant(out); checks = []
    try:
        with navigation(out, terrain, plant):
            cases = [('recorded goal3 pose', (4.52, -5.24, -math.pi/2)), ('recorded goal4 pose', (4.59, -5.27, -math.pi/2))] if args.recorded_exit else [('overlapping corner', (-.8, 1.1, 0.)), ('occupied centre inside wall', (-.1, 1.1, 0.))]
            for label, start in cases:
                plant.pose = np.array(start); plant.run_for(1.2)
                begin = len(plant.samples)
                goal = (6.45, -6.13, 0.) if args.recorded_exit else (-2.2, 1.1, 0.)
                plant.goal(goal, 45.)
                trace = plant.samples[begin:]
                escape = [s for s in trace if np.linalg.norm(s['cmd'][:2]) > .01 and (np.linalg.norm(np.array(s['pose'][:2])-start[:2]) < .08 if args.recorded_exit else abs(s['pose'][0]) < .95)]
                assert escape, 'no escape movement'
                assert max(abs(s['cmd'][2]) for s in escape) < .01, 'escape rotated while embedded'
                assert max(np.linalg.norm(s['cmd'][:2]) for s in escape) <= .16, 'escape exceeded speed limit'
                checks.append(label+' escapes without rotation and reaches target')
                print('PASS '+checks[-1], flush=True)
                plant.goal((6.47, -5.88, 0.) if args.recorded_exit else (-2.7, .9, .4))
                checks.append('fresh goal after '+label+' succeeds')
                print('PASS '+checks[-1], flush=True)
            text = (out/'navigation.log').read_text()
            assert 'occupied start' in text and 'occupied footprint, escaping' in text
            result = dict(passed=True, checks=checks)
    except Exception:
        result = dict(passed=False, checks=checks, error=traceback.format_exc())
        print(result['error'], flush=True)
    finally:
        plant.trace.close(); plant.destroy_node(); rclpy.shutdown()
    return finish(out, result)
if __name__ == '__main__':
    raise SystemExit(main())
