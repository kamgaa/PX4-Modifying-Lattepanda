"""A single guarded boundary for all command publishers, testable without a graph."""
from math import atan2, sin, cos

COMMAND_TOPICS = ('/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint',
                  '/fmu/in/vehicle_command', '/j1_angle_cmd')


def command_topics(dry_run):
    return () if dry_run else COMMAND_TOPICS


def emit(dry_run, output, stamp, publishers, ids, attempt):
    if dry_run or output.target is None:
        return
    from .messages import position_messages, land_command
    from std_msgs.msg import Float32
    target = output.target
    mode, goal = position_messages(stamp, target.position, atan2(sin(target.yaw), cos(target.yaw)))
    publishers['goal'].publish(goal)
    publishers['mode'].publish(mode)
    if output.joint_command is not None:
        msg = Float32()
        msg.data = float(output.joint_command)
        publishers['joint'].publish(msg)
    if output.request_land:
        publishers['command'].publish(land_command(stamp, *ids, attempt))
