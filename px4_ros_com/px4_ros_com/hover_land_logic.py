"""One-shot position-only mission. No ROS imports, I/O, or vehicle commands."""

from dataclasses import dataclass
from math import dist, isfinite
from typing import Optional, Tuple

XYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class Settings:
    hover_height_m: float = 1.0
    hover_duration_s: float = 10.0
    climb_rate_mps: float = 0.2
    publish_rate_hz: float = 20.0
    position_tolerance_m: float = 0.1
    arrival_settle_time_s: float = 1.0
    prestream_duration_s: float = 1.5
    position_timeout_s: float = 0.5
    status_timeout_s: float = 1.0
    climb_timeout_s: float = 30.0
    land_retry_interval_s: float = 2.0
    land_max_attempts: int = 3
    max_step_dt_s: float = 0.1
    control_timeout_s: float = 0.3

    def __post_init__(self):
        for name, value in vars(self).items():
            if not isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if type(self.land_max_attempts) is not int or self.land_max_attempts > 10:
            raise ValueError('land_max_attempts must be an integer in [1, 10]')
        if not 5.0 <= self.publish_rate_hz <= 100.0:
            raise ValueError('publish_rate_hz must be in [5, 100]')
        if self.prestream_duration_s <= 1.0:
            raise ValueError('prestream_duration_s must exceed 1 second')
        if not 1.0 / self.publish_rate_hz <= self.max_step_dt_s < self.control_timeout_s:
            raise ValueError('require period <= max_step_dt_s < control_timeout_s')
        minimum = self.hover_height_m / self.climb_rate_mps + self.arrival_settle_time_s
        if self.climb_timeout_s <= minimum:
            raise ValueError('climb_timeout_s is shorter than climb plus settling')


@dataclass(frozen=True)
class Position:
    stamp: int
    sample_stamp: int
    xyz: XYZ
    yaw: float
    xy_valid: bool
    z_valid: bool
    heading_valid: bool
    resets: Tuple[int, int, int, int]

    @property
    def valid(self):
        return (self.stamp > 0 and self.sample_stamp > 0
                and self.xy_valid and self.z_valid and self.heading_valid
                and all(isfinite(v) for v in (*self.xyz, self.yaw)))


@dataclass(frozen=True)
class Status:
    stamp: int
    armed: bool
    disarmed: bool
    offboard: bool
    auto_land: bool
    healthy: bool = True


@dataclass(frozen=True)
class RelativeClimb:
    start: XYZ
    yaw: float
    height: float

    def target(self, ascent: float) -> XYZ:
        return (self.start[0], self.start[1],
                self.start[2] - min(self.height, max(0.0, ascent)))

    @property
    def final(self) -> XYZ:
        return self.target(self.height)


@dataclass(frozen=True)
class Output:
    target: Optional[XYZ] = None
    yaw: float = 0.0
    request_land: bool = False


class HoverLand:
    STOPPED = {'ABORTED', 'AUTO_LAND_HANDOVER'}
    ACTIVE = {'CLIMB', 'HOLD', 'LAND_REQUESTED', 'LAND_FAILED_HOLD'}

    def __init__(self, settings: Settings):
        self.p = settings
        self.state = 'WAIT_INPUT'
        self.reason = 'Waiting for valid position, heading and status'
        self.position = None
        self.status = None
        self.position_at = None
        self.status_at = None
        self.seen_disarmed = False
        self.seen_offboard = False
        self.streaming = False
        self.prestream_at = None
        self.last_tick = None
        self.path = None
        self.ascent = 0.0
        self.climb_at = None
        self.settle_at = None
        self.hold_at = None
        self.attempts = 0
        self.last_land_at = None
        self.last_ack = None

    def abort(self, reason: str):
        if self.state not in self.STOPPED:
            self.state, self.reason = 'ABORTED', reason

    def _gap(self, now, previous, timeout):
        return previous is not None and now - previous > timeout

    def receive_position(self, now: float, value: Position):
        if self.state in self.STOPPED:
            return
        old = self.position
        if self.streaming and self._gap(now, self.position_at, self.p.position_timeout_s):
            self.abort('Position reception gap')
        if old is not None:
            if value.stamp < old.stamp or value.sample_stamp < old.sample_stamp:
                self.abort('Position timestamp regressed / PX4 restarted')
                return
            if self.streaming and value.resets != old.resets:
                self.abort('EKF position/heading reset or local reference change')
                return
        if not value.valid:
            if self.streaming:
                self.abort('Invalid position or heading')
            self.position = value
            return
        if old is not None and (value.stamp == old.stamp
                                or value.sample_stamp == old.sample_stamp):
            return  # A repeated sample must not refresh freshness.
        self.position, self.position_at = value, now

    def receive_status(self, now: float, value: Status):
        if self.state in self.STOPPED:
            return
        if self.streaming and self._gap(now, self.status_at, self.p.status_timeout_s):
            self.abort('Status reception gap')
        if self.status is not None:
            if value.stamp < self.status.stamp:
                self.abort('Status timestamp regressed / PX4 restarted')
                return
            if value.stamp == self.status.stamp:
                return
        self.status, self.status_at = value, now
        if value.stamp <= 0 or not value.healthy or not (value.armed or value.disarmed):
            if self.streaming:
                self.abort('Invalid vehicle status, failsafe, or unexpected system ID')
            return
        if value.disarmed:
            self.seen_disarmed = True
        if self.state in self.ACTIVE:
            if value.disarmed:
                self.abort('Disarmed: mission will not restart')
            elif not value.offboard:
                if value.auto_land and self.state in {'LAND_REQUESTED', 'LAND_FAILED_HOLD'}:
                    self.state = 'AUTO_LAND_HANDOVER'
                    self.reason = 'AUTO_LAND observed; landing completion is NOT monitored'
                else:
                    self.abort('RC/QGC/PX4 left Offboard; no mode or Land retries')
        elif self.streaming:
            if self.seen_offboard and not value.offboard:
                self.abort('Left Offboard before mission start')
            if value.offboard:
                self.seen_offboard = True
            if value.armed and self.state != 'READY':
                self.abort('Armed before READY; restart only after disarming')

    def receive_ack(self, now: float, result: int):
        # ACK is diagnostic only. Actual VehicleStatus owns every mode transition.
        if self.state == 'LAND_REQUESTED' and self.last_land_at is not None:
            if 0.0 <= now - self.last_land_at <= self.p.land_retry_interval_s:
                self.last_ack = result

    def _fresh(self, now):
        return (self.position is not None and self.position.valid
                and self.status is not None and self.status.stamp > 0
                and self.status.healthy and (self.status.armed or self.status.disarmed)
                and self.position_at is not None and self.status_at is not None
                and 0.0 <= now - self.position_at <= self.p.position_timeout_s
                and 0.0 <= now - self.status_at <= self.p.status_timeout_s)

    def tick(self, now: float) -> Output:
        if self.state in self.STOPPED:
            return Output()

        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now

        if dt < 0.0 or (self.streaming and dt > self.p.control_timeout_s):
            self.abort('Control loop stalled or monotonic time regressed')
            return Output()

        if not self._fresh(now):
            if self.streaming:
                pos_age = (
                    now - self.position_at
                    if self.position_at is not None else float('inf')
                )
                status_age = (
                    now - self.status_at
                    if self.status_at is not None else float('inf')
                )
                self.abort(
                    'Position/status timeout or invalid input: '
                    f'pos_age={pos_age:.3f}s/{self.p.position_timeout_s:.3f}s, '
                    f'status_age={status_age:.3f}s/{self.p.status_timeout_s:.3f}s, '
                    f'pos_valid={self.position is not None and self.position.valid}, '
                    f'status_healthy={self.status is not None and self.status.healthy}'
                )
            return Output()


    #if not self._fresh(now):
        #    if self.streaming:
        #        self.abort('Position/status timeout or invalid input')
        #    return Output()
        if not self.seen_disarmed:
            self.state, self.reason = 'WAIT_DISARM', 'Observe DISARMED after node startup first'
            return Output()
        if self.state in {'WAIT_INPUT', 'WAIT_DISARM'}:
            if self.status.armed:
                self.abort('Armed before prestream')
                return Output()
            self.state, self.reason = 'PRESTREAM', 'Sending current pose; do not Arm yet'
            self.streaming = True
            self.prestream_at = now
            self.seen_offboard = self.status.offboard
        if self.state == 'PRESTREAM' and now - self.prestream_at >= self.p.prestream_duration_s:
            self.state, self.reason = 'READY', 'READY: select Offboard and Arm using QGC'
        if self.state == 'READY' and self.status.armed and self.status.offboard:
            self.path = RelativeClimb(self.position.xyz, self.position.yaw, self.p.hover_height_m)
            self.climb_at, self.ascent, dt = now, 0.0, 0.0
            self.state, self.reason = 'CLIMB', 'Captured start XYZ/yaw once; relative ascent'
        if self.state in {'PRESTREAM', 'READY'}:
            return Output(self.position.xyz, self.position.yaw)
        if self.state == 'CLIMB':
            if now - self.climb_at >= self.p.climb_timeout_s:
                self.abort('Arrival timeout; Offboard stream stopped, pilot/failsafe takes over')
                return Output()
            self.ascent = min(self.p.hover_height_m,
                              self.ascent + self.p.climb_rate_mps * min(dt, self.p.max_step_dt_s))
            arrived = (self.ascent >= self.p.hover_height_m
                       and dist(self.position.xyz, self.path.final) <= self.p.position_tolerance_m)
            if not arrived:
                self.settle_at = None
            elif self.settle_at is None:
                self.settle_at = now
            elif now - self.settle_at >= self.p.arrival_settle_time_s:
                self.state, self.reason, self.hold_at = 'HOLD', 'Settled; timed hover started', now
        elif self.state == 'HOLD':
            if dist(self.position.xyz, self.path.final) > self.p.position_tolerance_m:
                self.state, self.reason = 'CLIMB', 'Outside tolerance; settle and hover timer reset'
                self.settle_at, self.hold_at = None, None
            elif now - self.hold_at >= self.p.hover_duration_s:
                self.state, self.reason = 'LAND_REQUESTED', 'Request Land; wait for AUTO_LAND'
        request = False
        if self.state == 'LAND_REQUESTED':
            due = self.last_land_at is None or now - self.last_land_at >= self.p.land_retry_interval_s
            if due and self.attempts < self.p.land_max_attempts:
                self.attempts += 1
                self.last_land_at, self.last_ack, request = now, None, True
            elif due:
                self.state = 'LAND_FAILED_HOLD'
                self.reason = 'Land attempts exhausted; holding goal while inputs valid; pilot intervention required'
        return Output(self.path.target(self.ascent), self.path.yaw, request)
