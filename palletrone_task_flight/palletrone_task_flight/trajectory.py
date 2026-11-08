"""One fixed contact point; scalar interpolation preserves commanded alignment."""
from dataclasses import dataclass
from math import radians, hypot, isfinite
from .geometry import Geometry, add, sub, scale, smoothstep, norm


@dataclass(frozen=True)
class Target:
    position: tuple
    yaw: float
    theta: float
    distance: float


@dataclass(frozen=True)
class Segment:
    name: str
    kind: str
    start: Target
    end: Target
    duration: float
    geometry: Geometry
    contact: tuple

    def sample(self, elapsed):
        s = smoothstep(elapsed / self.duration)
        yaw = self.start.yaw + s*(self.end.yaw-self.start.yaw)
        theta = self.start.theta + s*(self.end.theta-self.start.theta)
        d = self.start.distance + s*(self.end.distance-self.start.distance)
        if self.kind == 'climb':
            position = add(self.start.position, scale(sub(self.end.position, self.start.position), s))
        else:
            position = self.geometry.target(self.contact, yaw, theta, d)
        return Target(position, yaw, theta, d)

    @property
    def is_hold(self):
        return self.kind == 'hold'


class Plan:
    def __init__(self, start, yaw, settings):
        if len(start)!=3 or not all(isfinite(v) for v in (*start,yaw)):
            raise ValueError('plan start requires finite XYZ and yaw')
        self.p = settings
        self.start = tuple(start)
        self.yaw0 = yaw
        self.geometry = Geometry(settings.rod_length_m, settings.px4_reference_to_bottom_frd_m,
                                 settings.j1_mount_from_bottom_frd_m)
        p = settings
        self.hover = add(start, (0.0, 0.0, -p.hover_height_m))
        theta1, theta2 = radians(p.pose1_angle_deg), radians(p.pose2_angle_deg)
        d = p.standoff_distance_m
        self.contact = self.geometry.contact(self.hover, yaw, theta1, d)
        self.segments = []
        self.current = Target(tuple(start), yaw, theta1, d)
        climb_time = 1.875*p.hover_height_m/min(p.climb_rate_mps, p.max_reference_speed_mps)
        self._append('CLIMB', 'climb', Target(self.hover, yaw, theta1, d), climb_time)
        if climb_time+p.segment_settle_time_s >= p.climb_timeout_s:
            raise ValueError('climb timeout must exceed rate-limited climb plus settling')
        self._hold('INITIAL_HOVER', p.initial_hover_duration_s)
        for pass_index, theta in enumerate((theta1, theta2), 1):
            if pass_index == 2:
                self._move('POSTURE_2', 'theta', theta, p.posture_change_duration_s)
            angles = p.azimuth_offsets_deg
            if pass_index == 2 and p.reverse_second_pass:
                angles = tuple(reversed(angles))
            for offset in angles:
                target_yaw = yaw+radians(offset)
                if abs(target_yaw-self.current.yaw) > 1e-12:
                    self._move(f'ORBIT_P{pass_index}_{offset:g}', 'yaw', target_yaw, p.orbit_duration_s)
                label = f'P{pass_index}_{offset:g}'
                self._move('APPROACH_'+label, 'distance', 0.0, p.approach_duration_s)
                self._hold('AT_CONTACT_TARGET_'+label, p.contact_hold_duration_s)
                self._move('RETRACT_'+label, 'distance', d, p.retract_duration_s)
        if abs(self.current.yaw-yaw) > 1e-12:
            self._move('RETURN_AZIMUTH', 'yaw', yaw, p.orbit_duration_s)
        self._move('RETURN_HORIZONTAL', 'theta', theta1, p.posture_change_duration_s)
        self._hold('FINAL_HOVER', p.final_hover_duration_s)
        self.nominal_duration = sum(s.duration + p.segment_settle_time_s for s in self.segments)
        if self.nominal_duration >= p.mission_timeout_s:
            raise ValueError('mission_timeout_s must exceed nominal plan including settling')

    def _append(self, name, kind, target, duration):
        self.segments.append(Segment(name, kind, self.current, target, duration, self.geometry, self.contact))
        self.current = target

    def _hold(self, name, duration):
        self._append(name, 'hold', self.current, duration)

    def _move(self, name, variable, value, requested_time):
        c, p = self.current, self.p
        yaw = value if variable == 'yaw' else c.yaw
        theta = value if variable == 'theta' else c.theta
        d = value if variable == 'distance' else c.distance
        target = Target(self.geometry.target(self.contact, yaw, theta, d), yaw, theta, d)
        # max ds/du = 1.875 for quintic smoothstep. Bounds cover the entire segment.
        dy, dt, dd = abs(yaw-c.yaw), abs(theta-c.theta), abs(d-c.distance)
        radius_bound = hypot(*self.geometry.mount[:2]) + self.geometry.length + max(d, c.distance)
        path_bound = dd + (self.geometry.length+max(d,c.distance))*dt + radius_bound*dy
        duration = max(requested_time, 1.875*path_bound/p.max_reference_speed_mps,
                       1.875*dy/radians(p.max_reference_yaw_rate_deg_s),
                       1.875*dt/radians(p.max_joint_rate_deg_s))
        self._append(name, variable, target, duration)

    def samples(self, steps=200):
        elapsed = 0.0
        for segment in self.segments:
            for i in range(steps+1):
                yield elapsed+segment.duration*i/steps, segment, segment.sample(segment.duration*i/steps)
            elapsed += segment.duration

    def summary(self):
        values = [target.position for _, _, target in self.samples(400)]
        return {'contact_ned_m': self.contact, 'hover_ned_m': self.hover,
                'nominal_duration_with_settling_s': self.nominal_duration,
                'max_rise_from_start_m': self.start[2]-min(v[2] for v in values),
                'max_horizontal_distance_from_start_m': max(norm(sub(v[:2], self.start[:2])) for v in values),
                'xyz_min_m': tuple(min(v[k] for v in values) for k in range(3)),
                'xyz_max_m': tuple(max(v[k] for v in values) for k in range(3)),
                'segments': [{'name':s.name,'duration_s':s.duration} for s in self.segments]}
