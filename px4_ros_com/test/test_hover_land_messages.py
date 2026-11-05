"""Message serialization checks only. Never initializes a node or publishes."""

from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from rclpy.serialization import deserialize_message, serialize_message
    from px4_msgs.msg import VehicleCommand
    from px4_ros_com.hover_land_messages import (
        check_publish_schemas, land_command, position_messages,
    )
    ROS_MESSAGES = True
except ImportError:
    ROS_MESSAGES = False


@unittest.skipUnless(ROS_MESSAGES, 'Source ROS 2 Humble and the audited px4_msgs installation')
class MessageTests(unittest.TestCase):
    def test_exact_schema(self):
        check_publish_schemas()

    def test_position_only_roundtrip(self):
        mode, goal = position_messages(123456789, (4.0, -3.0, 1.0), 0.7)
        mode = deserialize_message(serialize_message(mode), type(mode))
        goal = deserialize_message(serialize_message(goal), type(goal))
        self.assertTrue(mode.position)
        for name in ['velocity', 'acceleration', 'attitude', 'body_rate',
                     'thrust_and_torque', 'direct_actuator']:
            self.assertFalse(getattr(mode, name))
        self.assertEqual(list(goal.position), [4.0, -3.0, 1.0])
        self.assertEqual(goal.timestamp, 123456789)
        self.assertAlmostEqual(goal.yaw, 0.7, places=6)
        for name in ['velocity', 'acceleration', 'jerk']:
            self.assertTrue(all(math.isnan(v) for v in getattr(goal, name)))
        self.assertTrue(math.isnan(goal.yawspeed))

    def test_no_global_land_coordinate_or_arming_command(self):
        msg = land_command(123456789, 1, 1, 245, 191, 3)
        msg = deserialize_message(serialize_message(msg), type(msg))
        self.assertEqual(msg.command, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.assertEqual((msg.target_system, msg.target_component), (1, 1))
        self.assertEqual((msg.source_system, msg.source_component), (245, 191))
        self.assertEqual(msg.confirmation, 2)
        self.assertTrue(msg.from_external)
        for name in ['param4', 'param5', 'param6', 'param7']:
            self.assertTrue(math.isnan(getattr(msg, name)))

    def test_invalid_goal_rejected(self):
        for stamp, xyz, yaw in [(0, (1.0, 2.0, 3.0), 0.0),
                                (1, (math.nan, 2.0, 3.0), 0.0),
                                (1, (1.0, 2.0, 3.0), math.nan)]:
            with self.assertRaises(ValueError):
                position_messages(stamp, xyz, yaw)


if __name__ == '__main__':
    unittest.main(verbosity=2)
