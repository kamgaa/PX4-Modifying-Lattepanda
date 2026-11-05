"""Message construction only: importing this module does not start ROS."""

from math import isfinite, nan

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def check_publish_schemas():
    expected = {
        OffboardControlMode: [
            ('timestamp', 'uint64'), ('position', 'boolean'), ('velocity', 'boolean'),
            ('acceleration', 'boolean'), ('attitude', 'boolean'), ('body_rate', 'boolean'),
            ('thrust_and_torque', 'boolean'), ('direct_actuator', 'boolean')],
        TrajectorySetpoint: [
            ('timestamp', 'uint64'), ('position', 'float[3]'), ('velocity', 'float[3]'),
            ('acceleration', 'float[3]'), ('jerk', 'float[3]'), ('yaw', 'float'),
            ('yawspeed', 'float')],
        VehicleCommand: [
            ('timestamp', 'uint64'), ('param1', 'float'), ('param2', 'float'),
            ('param3', 'float'), ('param4', 'float'), ('param5', 'double'),
            ('param6', 'double'), ('param7', 'float'), ('command', 'uint32'),
            ('target_system', 'uint8'), ('target_component', 'uint8'),
            ('source_system', 'uint8'), ('source_component', 'uint16'),
            ('confirmation', 'uint8'), ('from_external', 'boolean')],
    }
    for cls, schema in expected.items():
        if list(cls.get_fields_and_field_types().items()) != schema:
            raise RuntimeError(f'Unexpected installed schema: {cls.__name__}; audit firmware/messages first')


def position_messages(stamp, xyz, yaw):
    if stamp <= 0 or len(xyz) != 3 or not all(isfinite(v) for v in (*xyz, yaw)):
        raise ValueError('Position/yaw must be finite and timestamp positive')
    mode = OffboardControlMode()
    mode.timestamp = int(stamp)
    mode.position = True
    mode.velocity = False
    mode.acceleration = False
    mode.attitude = False
    mode.body_rate = False
    mode.thrust_and_torque = False
    mode.direct_actuator = False
    goal = TrajectorySetpoint()
    goal.timestamp = int(stamp)
    goal.position = [float(v) for v in xyz]
    goal.velocity = [nan, nan, nan]
    goal.acceleration = [nan, nan, nan]
    goal.jerk = [nan, nan, nan]
    goal.yaw = float(yaw)
    goal.yawspeed = nan
    return mode, goal


def land_command(stamp, target_system, target_component, source_system, source_component, attempt):
    msg = VehicleCommand()
    msg.timestamp = int(stamp)
    msg.param1 = 0.0
    msg.param2 = 0.0
    msg.param3 = 0.0
    msg.param4 = nan
    msg.param5 = nan
    msg.param6 = nan
    msg.param7 = nan
    msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
    msg.target_system = target_system
    msg.target_component = target_component
    msg.source_system = source_system
    msg.source_component = source_component
    msg.confirmation = attempt - 1
    msg.from_external = True
    return msg
