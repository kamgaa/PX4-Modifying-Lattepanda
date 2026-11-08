"""One-shot task state machine. No ROS imports, hardware access, or publishers."""
from dataclasses import dataclass
from math import isfinite, radians
from .geometry import norm, sub, angle_error
from .trajectory import Plan, Target


@dataclass(frozen=True)
class Position:
    stamp: int
    sample_stamp: int
    xyz: tuple
    yaw: float
    xy_valid: bool
    z_valid: bool
    heading_good: bool
    resets: tuple


@dataclass(frozen=True)
class Status:
    stamp: int
    armed: bool
    disarmed: bool
    offboard: bool
    auto_land: bool
    healthy: bool = True


@dataclass(frozen=True)
class Joint:
    stamp: int
    theta: float
    ready: bool


@dataclass(frozen=True)
class Output:
    target: object = None
    joint_command: object = None
    request_land: bool = False


class TaskMachine:
    STOPPED = {'ABORTED', 'AUTO_LAND_HANDOVER'}
    ACTIVE = {'RUNNING', 'LAND_REQUESTED', 'LAND_FAILED_HOLD', 'FAULT_HOLD'}

    def __init__(self, settings):
        self.p = settings
        self.state = 'WAIT_INPUT'
        self.reason = 'Waiting for position/status/J1 and post-start DISARMED'
        self.position = self.status = self.joint = None
        self.position_at = self.status_at = self.joint_at = None
        self.last_tick = None
        self.streaming = self.seen_disarmed = self.seen_offboard = False
        self.ready_at = self.prestream_at = None
        self.plan = None
        self.last_target = None
        self.index = 0
        self.elapsed = 0.0
        self.segment_at = self.mission_at = self.settle_at = None
        self.joint_lag_at = None
        self.land_attempts = 0
        self.last_land_at = None
        self.last_ack = None

    def abort(self, reason):
        if self.state not in self.STOPPED:
            self.state, self.reason = 'ABORTED', reason

    def fault_hold(self, reason):
        if self.state in self.STOPPED or self.state == 'FAULT_HOLD':
            return
        if self.state in self.ACTIVE and self.last_target is not None:
            self.state, self.reason = 'FAULT_HOLD', reason+'; pilot takeover required; no automatic Land'
        elif self.streaming:
            self.abort(reason)

    def _position_valid(self, value):
        return (value is not None and value.stamp > 0 and value.sample_stamp > 0
                and value.xy_valid and value.z_valid
                and (value.heading_good or not self.p.require_heading_good_for_control)
                and len(value.xyz) == 3 and all(isfinite(x) for x in (*value.xyz, value.yaw)))

    def receive_position(self, now, value):
        if self.state in self.STOPPED:
            return
        if self.streaming and not self._age_ok(now, self.position_at, self.p.position_timeout_s):
            self.abort('Position reception gap')
        if self.position is not None:
            if value.stamp < self.position.stamp or value.sample_stamp < self.position.sample_stamp:
                self.abort('PX4 position timestamp regressed')
                return
            if self.streaming and value.resets != self.position.resets:
                self.abort('EKF position/heading/reference reset; no coordinate compensation')
                return
        if not self._position_valid(value):
            self.position = value
            if self.streaming:
                self.abort('Invalid position or nonfinite heading')
            return
        if self.position is not None and (value.stamp == self.position.stamp or value.sample_stamp == self.position.sample_stamp):
            return
        self.position, self.position_at = value, now

    def receive_status(self, now, value):
        if self.state in self.STOPPED:
            return
        if self.streaming and not self._age_ok(now, self.status_at, self.p.status_timeout_s):
            self.abort('Status reception gap')
        if self.status is not None:
            if value.stamp < self.status.stamp:
                self.abort('PX4 status timestamp regressed')
                return
            if value.stamp == self.status.stamp:
                return
        self.status, self.status_at = value, now
        if value.stamp <= 0 or not value.healthy or not (value.armed or value.disarmed):
            if self.streaming:
                self.abort('Invalid vehicle status/failsafe/system ID')
            return
        if value.disarmed:
            self.seen_disarmed = True
        if self.state in self.ACTIVE:
            if value.disarmed:
                self.abort('Disarmed; mission latched off')
            elif not value.offboard:
                if value.auto_land and self.state in {'LAND_REQUESTED', 'LAND_FAILED_HOLD'}:
                    self.state, self.reason = 'AUTO_LAND_HANDOVER', 'AUTO_LAND confirmed; touchdown NOT observed'
                else:
                    self.abort('RC/QGC/PX4 mode takeover; no further Land requests')
        elif self.streaming:
            if self.seen_offboard and not value.offboard:
                self.abort('Left Offboard during preparation')
            self.seen_offboard = self.seen_offboard or value.offboard
            if value.armed and self.state != 'READY':
                self.abort('Armed before READY')

    def receive_joint(self, now, value):
        if self.state in self.STOPPED:
            return
        if self.streaming and not self._age_ok(now, self.joint_at, self.p.joint_feedback_timeout_s):
            self.fault_hold('J1 feedback reception gap')
        if self.joint is not None:
            if value.stamp < self.joint.stamp:
                self.fault_hold('J1 timestamp regressed')
                return
            if value.stamp == self.joint.stamp:
                return
        self.joint, self.joint_at = value, now
        if not self._joint_valid(value) and self.streaming:
            self.fault_hold('J1 invalid or calibration/readiness lost')

    @staticmethod
    def _age_ok(now, previous, timeout):
        return previous is not None and 0 <= now-previous <= timeout

    @staticmethod
    def _joint_valid(value):
        return value is not None and value.stamp > 0 and value.ready and isfinite(value.theta)

    def px4_fresh(self, now):
        return (self._position_valid(self.position) and self.status is not None
                and self.status.stamp > 0 and self.status.healthy
                and (self.status.armed or self.status.disarmed)
                and self._age_ok(now, self.position_at, self.p.position_timeout_s)
                and self._age_ok(now, self.status_at, self.p.status_timeout_s))

    def joint_fresh(self, now):
        return self._joint_valid(self.joint) and self._age_ok(now, self.joint_at, self.p.joint_feedback_timeout_s)

    def at_target(self, target):
        return (norm(sub(self.position.xyz, target.position)) <= self.p.position_tolerance_m
                and abs(angle_error(self.position.yaw, target.yaw)) <= radians(self.p.yaw_tolerance_deg)
                and abs(self.joint.theta-target.theta) <= radians(self.p.joint_tolerance_deg))

    def receive_ack(self, result):
        if self.state == 'LAND_REQUESTED':
            self.last_ack = result  # Diagnostics only, not mode confirmation.

    @property
    def phase(self):
        return self.plan.segments[self.index].name if self.state == 'RUNNING' else self.state

    def tick(self, now):
        if self.p.dry_run or self.state in self.STOPPED:
            return Output()
        dt = 0.0 if self.last_tick is None else now-self.last_tick
        self.last_tick = now
        if dt < 0 or (self.streaming and dt > self.p.control_timeout_s):
            self.abort('Control loop gap / monotonic clock regression')
            return Output()
        if not self.px4_fresh(now):
            if self.streaming:
                self.abort('Position/status timeout or invalid input')
            return Output()
        if not self.joint_fresh(now):
            if self.streaming:
                self.fault_hold('J1 feedback timeout or not ready')
            return self._output()
        if self.state == 'FAULT_HOLD':
            return self._output()
        if not self.seen_disarmed:
            self.state, self.reason = 'WAIT_DISARM', 'Observe DISARMED before starting'
            return Output()
        if self.state in {'WAIT_INPUT', 'WAIT_DISARM'}:
            if self.status.armed:
                self.abort('Armed before prestream')
                return Output()
            if abs(self.joint.theta-radians(self.p.pose1_angle_deg)) > radians(self.p.joint_tolerance_deg):
                self.reason = 'J1 must be calibrated and already in horizontal ready posture'
                return Output()
            try:
                self.plan = Plan(self.position.xyz, self.position.yaw, self.p)
            except ValueError as e:
                self.abort('Plan validation: '+str(e))
                return Output()
            self.streaming = True
            self.state, self.reason = 'PRESTREAM', 'Current pose prestream; do not Arm yet'
            self.prestream_at = now
            self.seen_offboard = self.status.offboard
        if self.state == 'PRESTREAM' and now-self.prestream_at >= self.p.prestream_duration_s:
            self.state, self.reason = 'READY', 'READY: QGC Offboard then Arm; do NOT use Takeoff'
        if self.state == 'READY' and self.status.armed and self.status.offboard:
            if abs(self.joint.theta-radians(self.p.pose1_angle_deg)) > radians(self.p.joint_tolerance_deg):
                self.abort('J1 not in ready posture at Arm')
                return Output()
            try:
                self.plan = Plan(self.position.xyz, self.position.yaw, self.p)
            except ValueError as e:
                self.abort('Final plan validation: '+str(e))
                return Output()
            self.mission_at = self.segment_at = now
            self.last_target = self.plan.segments[0].start
            self.state, self.reason, dt = 'RUNNING', 'Captured P0/yaw0 and fixed contact C once', 0.0
        if self.state in {'PRESTREAM', 'READY'}:
            if abs(self.joint.theta-radians(self.p.pose1_angle_deg)) > radians(self.p.joint_tolerance_deg):
                self.abort('J1 left ready posture during prestream')
                return Output()
            self.last_target = Target(self.position.xyz, self.position.yaw, radians(self.p.pose1_angle_deg), self.p.standoff_distance_m)
            return self._output()
        if self.state == 'RUNNING':
            self._run(now, min(dt, self.p.max_step_dt_s))
        if self.state in {'LAND_REQUESTED', 'LAND_FAILED_HOLD'}:
            if abs(self.joint.theta-radians(self.p.pose1_angle_deg)) > radians(self.p.joint_tolerance_deg):
                self.fault_hold('Horizontal landing posture lost')
                return self._output()
        request = False
        if self.state == 'LAND_REQUESTED':
            due = self.last_land_at is None or now-self.last_land_at >= self.p.land_retry_interval_s
            if due and self.land_attempts < self.p.land_max_attempts:
                self.land_attempts += 1
                self.last_land_at, request = now, True
            elif due:
                self.state, self.reason = 'LAND_FAILED_HOLD', 'Land retries exhausted; pilot intervention required'
        return self._output(request)

    def _run(self, now, dt):
        if now-self.mission_at > self.p.mission_timeout_s:
            self.fault_hold('Whole mission timeout')
            return
        segment = self.plan.segments[self.index]
        timeout = self.p.climb_timeout_s if segment.kind == 'climb' else segment.duration+self.p.segment_arrival_timeout_s
        if now-self.segment_at > timeout:
            self.fault_hold('Segment arrival timeout: '+segment.name)
            return
        joint_ok = abs(self.joint.theta-self.last_target.theta) <= radians(self.p.joint_tolerance_deg)
        if not joint_ok:
            if self.joint_lag_at is None:
                self.joint_lag_at = now
            elif now-self.joint_lag_at >= self.p.joint_tracking_timeout_s:
                self.fault_hold('J1 tracking error')
        else:
            self.joint_lag_at = None
        if not self.at_target(self.last_target):
            self.settle_at = None
            if segment.is_hold:
                self.elapsed = 0.0  # A hold must be continuous, not accumulated in fragments.
            return
        self.elapsed = min(segment.duration, self.elapsed+dt)
        self.last_target = segment.sample(self.elapsed)
        if self.elapsed < segment.duration or not self.at_target(self.last_target):
            self.settle_at = None
            return
        if self.settle_at is None:
            self.settle_at = now
        elif now-self.settle_at >= self.p.segment_settle_time_s:
            if self.index == len(self.plan.segments)-1:
                self.state, self.reason = 'LAND_REQUESTED', 'Final hover and horizontal J1 confirmed; requesting Land'
            else:
                self.index += 1
                self.segment_at, self.elapsed, self.settle_at = now, 0.0, None

    def _output(self, request=False):
        if self.state in self.STOPPED or self.last_target is None or self.p.dry_run:
            return Output()
        joint_command = None if self.state == 'FAULT_HOLD' else self.last_target.theta
        return Output(self.last_target, joint_command, request)
