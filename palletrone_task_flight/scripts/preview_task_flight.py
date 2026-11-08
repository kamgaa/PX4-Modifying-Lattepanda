#!/usr/bin/env python3
"""Offline executable: no rclpy import, no ROS graph, no serial device."""
import argparse
import json
from palletrone_task_flight.configuration import Settings,load_yaml
from palletrone_task_flight.trajectory import Plan
from palletrone_task_flight.preview import write_preview
from math import radians


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--start',nargs=3,type=float,default=[0,0,0],metavar=('X','Y','Z'))
    parser.add_argument('--yaw-deg',type=float,default=0)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    settings=load_yaml(args.config) if args.config else Settings()
    plan=Plan(tuple(args.start),radians(args.yaw_deg),settings)
    print(json.dumps(write_preview(plan,args.output),indent=2))


if __name__=='__main__':
    main()
