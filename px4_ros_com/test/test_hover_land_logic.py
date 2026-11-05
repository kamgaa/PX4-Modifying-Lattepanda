"""Run with python3 -B test/test_hover_land_logic.py; no ROS graph required."""

from dataclasses import replace
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from px4_ros_com.hover_land_logic import HoverLand, Position, Settings, Status


class Rig:
    def __init__(self, settings=None):
        self.core = HoverLand(settings or Settings())
        self.now = 100.0
        self.counter = 0
        self.xyz = (4.0, -3.0, 2.0)
        self.yaw = 0.7
        self.armed = False
        self.offboard = False
        self.auto_land = False
        self.resets = (0, 0, 0, 123)
        self.requests = []

    def step(self, dt=0.05, position=True, status=True, valid=True, follow=False):
        self.now += dt
        self.counter += 1
        stamp = self.counter * 50000
        if follow and self.core.path is not None:
            self.xyz = self.core.path.target(self.core.ascent)
        if position:
            self.core.receive_position(self.now, Position(
                stamp, stamp, self.xyz, self.yaw, valid, valid, valid, self.resets))
        if status:
            self.core.receive_status(self.now, Status(
                stamp, self.armed, not self.armed, self.offboard, self.auto_land))
        result = self.core.tick(self.now)
        if result.request_land:
            self.requests.append(self.now)
        return result

    def ready(self):
        for _ in range(40):
            self.step()
        assert self.core.state == 'READY', self.core.reason

    def start(self):
        self.ready()
        self.armed, self.offboard = True, True
        return self.step()

    def reach(self, wanted, limit=1000):
        for _ in range(limit):
            result = self.step(follow=True)
            if self.core.state == wanted:
                return result
        raise AssertionError((wanted, self.core.state, self.core.reason))


class HoverLandTests(unittest.TestCase):
    def test_no_default_zero_position(self):
        r = Rig()
        for _ in range(80):
            self.assertIsNone(r.step(position=False).target)
        self.assertFalse(r.core.streaming)

    def test_invalid_input_cannot_prepare(self):
        r = Rig()
        for _ in range(80):
            self.assertIsNone(r.step(valid=False).target)
        r.xyz = (math.nan, 0.0, 0.0)
        self.assertIsNone(r.step().target)

    def test_startup_armed_requires_disarmed_observation(self):
        r = Rig()
        r.armed, r.offboard = True, True
        for _ in range(80):
            self.assertIsNone(r.step().target)
        self.assertEqual(r.core.state, 'WAIT_DISARM')
        r.armed, r.offboard = False, False
        r.ready()
        r.armed, r.offboard = True, True
        self.assertEqual(r.step().target, (4.0, -3.0, 2.0))

    def test_both_armed_and_offboard_required(self):
        for armed, offboard in [(False, False), (False, True), (True, False)]:
            with self.subTest(armed=armed, offboard=offboard):
                r = Rig()
                r.ready()
                r.armed, r.offboard = armed, offboard
                r.step()
                self.assertEqual(r.core.state, 'READY')
                self.assertIsNone(r.core.path)

    def test_prestream_exceeds_one_second_and_early_arm_aborts(self):
        r = Rig()
        for _ in range(22):
            r.step()
        self.assertEqual(r.core.state, 'PRESTREAM')
        r.armed, r.offboard = True, True
        self.assertIsNone(r.step().target)
        self.assertEqual(r.core.state, 'ABORTED')

    def test_relative_target_xy_yaw_and_ascent_limit(self):
        r = Rig()
        r.start()
        self.assertEqual(r.core.path.final, (4.0, -3.0, 1.0))
        r.xyz, r.yaw = (12.0, 8.0, 1.2), -2.0
        previous_z = 2.0
        for _ in range(140):
            out = r.step()
            self.assertEqual(out.target[:2], (4.0, -3.0))
            self.assertEqual(out.yaw, 0.7)
            self.assertGreaterEqual(out.target[2], 1.0)
            self.assertLessEqual(previous_z - out.target[2], 0.010001)
            previous_z = out.target[2]
        self.assertAlmostEqual(previous_z, 1.0)

    def test_scheduler_delay_capped_and_large_stall_aborts(self):
        r = Rig()
        start = r.start().target[2]
        self.assertAlmostEqual(start - r.step(dt=0.2).target[2], 0.02)
        self.assertIsNone(r.step(dt=0.31).target)
        self.assertEqual(r.core.state, 'ABORTED')

    def test_settling_and_ten_second_hold_use_actual_position(self):
        r = Rig()
        r.start()
        for _ in range(140):
            r.step()  # Target arrives, measured position stays on ground.
        self.assertEqual(r.core.state, 'CLIMB')
        r.xyz = r.core.path.final
        for _ in range(18):
            r.step()
        self.assertEqual(r.core.state, 'CLIMB')
        r.reach('HOLD')
        began = r.core.hold_at
        while r.now - began < 9.9:
            self.assertFalse(r.step(follow=True).request_land)
        r.reach('LAND_REQUESTED')
        self.assertGreaterEqual(r.requests[0] - began, 10.0)

    def test_leaving_tolerance_resets_hold(self):
        r = Rig()
        r.start()
        r.reach('HOLD')
        r.xyz = (5.0, -3.0, 1.0)
        r.step()
        self.assertEqual(r.core.state, 'CLIMB')
        self.assertIsNone(r.core.hold_at)

    def test_ack_is_not_mode_transition(self):
        r = Rig()
        r.start()
        r.reach('LAND_REQUESTED')
        r.core.receive_ack(r.now, 0)
        self.assertEqual(r.core.state, 'LAND_REQUESTED')
        self.assertIsNotNone(r.step().target)
        r.offboard, r.auto_land = False, True
        self.assertIsNone(r.step().target)
        self.assertEqual(r.core.state, 'AUTO_LAND_HANDOVER')
        r.offboard, r.auto_land = True, False
        self.assertIsNone(r.step().target)

    def test_denied_missing_and_in_progress_ack_have_bounded_retries(self):
        for result in [None, 0, 1, 2, 3, 4, 5, 6]:
            with self.subTest(result=result):
                r = Rig()
                r.start()
                r.reach('LAND_REQUESTED')
                for _ in range(150):
                    if result is not None:
                        r.core.receive_ack(r.now, result)
                    out = r.step(follow=True)
                self.assertEqual(len(r.requests), 3)
                self.assertTrue(all(b - a >= 2.0 for a, b in zip(r.requests, r.requests[1:])))
                self.assertEqual(r.core.state, 'LAND_FAILED_HOLD')
                self.assertIsNotNone(out.target)
                self.assertFalse(out.request_land)

    def test_rc_handover_during_land_stops_retries_and_cannot_restart(self):
        r = Rig()
        r.start()
        r.reach('LAND_REQUESTED')
        r.offboard = False
        self.assertIsNone(r.step().target)
        r.offboard = True
        for _ in range(100):
            self.assertIsNone(r.step().target)
        self.assertEqual(len(r.requests), 1)

    def test_disarm_latches_abort(self):
        r = Rig()
        r.start()
        r.armed = False
        self.assertIsNone(r.step().target)
        r.armed = True
        self.assertIsNone(r.step().target)

    def test_input_loss_invalid_heading_and_resets_abort(self):
        for fault in ['position_loss', 'status_loss', 'invalid', 'xy', 'z', 'heading', 'reference']:
            with self.subTest(fault=fault):
                r = Rig()
                r.start()
                if fault in ['xy', 'z', 'heading', 'reference']:
                    reset = list(r.resets)
                    reset[['xy', 'z', 'heading', 'reference'].index(fault)] += 1
                    r.resets = tuple(reset)
                for _ in range(30):
                    out = r.step(position=fault != 'position_loss', status=fault != 'status_loss',
                                 valid=fault != 'invalid')
                self.assertIsNone(out.target)
                self.assertEqual(r.core.state, 'ABORTED')

    def test_duplicate_samples_do_not_refresh_freshness(self):
        r = Rig()
        r.start()
        old = r.core.position
        for _ in range(20):
            r.core.receive_position(r.now, old)
            out = r.step(position=False)
        self.assertIsNone(out.target)
        self.assertEqual(r.core.state, 'ABORTED')

    def test_timestamp_regression_latches_abort(self):
        r = Rig()
        r.start()
        r.core.receive_position(r.now, replace(r.core.position, sample_stamp=1))
        self.assertIsNone(r.step().target)

    def test_each_position_validity_condition_is_required(self):
        for change in [{'xy_valid': False}, {'z_valid': False}, {'heading_valid': False},
                       {'yaw': math.nan}, {'xyz': (0.0, math.inf, 0.0)}]:
            with self.subTest(change=change):
                r = Rig()
                r.start()
                r.core.receive_position(r.now, replace(r.core.position, **change))
                self.assertEqual(r.core.state, 'ABORTED')

    def test_delayed_reception_cannot_hide_gap_before_timer_runs(self):
        r = Rig()
        r.start()
        r.now += 0.6
        # Simulate a queued fresh callback arriving before the timer callback.
        old = r.core.position
        r.core.receive_position(r.now, replace(old, stamp=old.stamp + 1,
                                               sample_stamp=old.sample_stamp + 1))
        self.assertEqual(r.core.state, 'ABORTED')

    def test_late_auto_land_after_retry_exhaustion_stops_hold(self):
        r = Rig()
        r.start()
        r.reach('LAND_REQUESTED')
        r.reach('LAND_FAILED_HOLD')
        r.offboard, r.auto_land = False, True
        self.assertIsNone(r.step().target)
        self.assertEqual(r.core.state, 'AUTO_LAND_HANDOVER')

    def test_arrival_timeout_stops_both_streams(self):
        r = Rig(Settings(climb_timeout_s=8.0))
        r.start()
        for _ in range(170):
            out = r.step()
        self.assertIsNone(out.target)
        self.assertEqual(r.core.state, 'ABORTED')
        self.assertEqual(len(r.requests), 0)

    def test_bad_settings_fail_closed(self):
        for values in [{'hover_height_m': float('nan')}, {'prestream_duration_s': 1.0},
                       {'land_max_attempts': 0}, {'publish_rate_hz': 2.0}, {'climb_timeout_s': 1.0}]:
            with self.assertRaises(ValueError):
                Settings(**values)


if __name__ == '__main__':
    unittest.main(verbosity=2)
