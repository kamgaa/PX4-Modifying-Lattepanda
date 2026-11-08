"""Strict parameter validation; no ROS dependencies."""
from dataclasses import dataclass, fields
from math import isfinite


@dataclass(frozen=True)
class Settings:
    dry_run: bool = True
    hover_height_m: float = 0.5
    initial_hover_duration_s: float = 10.0
    climb_rate_mps: float = 0.2
    rod_length_m: float = 0.60
    px4_reference_to_bottom_frd_m: tuple = (0.0, 0.0, 0.0)
    j1_mount_from_bottom_frd_m: tuple = (-0.11, 0.0, 0.0)
    standoff_distance_m: float = 0.30
    pose1_angle_deg: float = 0.0
    pose2_angle_deg: float = 30.0
    azimuth_offsets_deg: tuple = (0.0, 30.0, 60.0)
    reverse_second_pass: bool = True
    approach_duration_s: float = 10.0
    retract_duration_s: float = 10.0
    orbit_duration_s: float = 10.0
    posture_change_duration_s: float = 10.0
    contact_hold_duration_s: float = 2.0
    segment_settle_time_s: float = 1.0
    final_hover_duration_s: float = 3.0
    publish_rate_hz: float = 20.0
    position_tolerance_m: float = 0.05
    yaw_tolerance_deg: float = 3.0
    joint_tolerance_deg: float = 2.0
    climb_timeout_s: float = 30.0
    segment_arrival_timeout_s: float = 30.0
    mission_timeout_s: float = 600.0
    position_timeout_s: float = 0.5
    status_timeout_s: float = 2.0
    joint_feedback_timeout_s: float = 0.5
    joint_tracking_timeout_s: float = 2.0
    control_timeout_s: float = 0.3
    max_step_dt_s: float = 0.1
    max_reference_speed_mps: float = 0.2
    max_reference_yaw_rate_deg_s: float = 10.0
    max_joint_rate_deg_s: float = 15.0
    require_heading_good_for_control: bool = False
    prestream_duration_s: float = 1.5
    discovery_wait_s: float = 2.0
    land_retry_interval_s: float = 2.0
    land_max_attempts: int = 3
    target_system_id: int = 1
    target_component_id: int = 1
    source_system_id: int = 245
    source_component_id: int = 191

    def __post_init__(self):
        nonpositive = {'pose1_angle_deg', 'pose2_angle_deg'}
        for f in fields(self):
            value = getattr(self, f.name)
            if f.type is bool:
                if type(value) is not bool:
                    raise ValueError(f'{f.name}: expected bool')
            elif f.type is int:
                if type(value) is not int or value <= 0:
                    raise ValueError(f'{f.name}: expected positive integer')
            elif f.type is tuple:
                if f.name == 'azimuth_offsets_deg':
                    if not isinstance(value, (tuple, list)) or not value:
                        raise ValueError(f'{f.name}: expected at least one angle')
                elif not isinstance(value, (tuple, list)) or len(value) != 3:
                    raise ValueError(f'{f.name}: expected three numbers')
                if not all(type(x) in (int, float) and isfinite(x) for x in value):
                    raise ValueError(f'{f.name}: expected finite numbers')
                object.__setattr__(self, f.name, tuple(value))
            elif type(value) not in (float, int) or not isfinite(value):
                raise ValueError(f'{f.name}: expected finite number')
            elif f.name not in nonpositive and value <= 0:
                raise ValueError(f'{f.name}: must be positive')
        if abs(self.pose1_angle_deg) > 1e-9:
            raise ValueError('pose1_angle_deg must be 0: horizontal landing posture')
        if not -60 <= self.pose2_angle_deg <= 60:
            raise ValueError('pose2 angle must be within [-60, 60] degrees')
        if len(set(self.azimuth_offsets_deg)) != len(self.azimuth_offsets_deg) or self.azimuth_offsets_deg[0] != 0:
            raise ValueError('three distinct azimuths starting at 0 required')
        if any(abs(x) > 180 for x in self.azimuth_offsets_deg):
            raise ValueError('azimuth offsets must be in [-180, 180]')
        if any(abs(x) > 2 for x in (*self.px4_reference_to_bottom_frd_m, *self.j1_mount_from_bottom_frd_m)):
            raise ValueError('offset magnitude exceeds 2m per component')
        if not 0 < self.rod_length_m <= 2 or not 0 < self.standoff_distance_m <= 2:
            raise ValueError('rod length and standoff must be in (0, 2] m')
        if not 0 < self.hover_height_m <= 5:
            raise ValueError('initial ascent must be in (0, 5] m')
        if not 5 <= self.publish_rate_hz <= 100 or self.prestream_duration_s <= 1:
            raise ValueError('publish rate 5..100 Hz and prestream >1s required')
        if not 1/self.publish_rate_hz <= self.max_step_dt_s < self.control_timeout_s:
            raise ValueError('require timer period <= max_step_dt < control_timeout')
        if self.position_timeout_s > 5 or self.status_timeout_s > 10 or self.joint_feedback_timeout_s > 2:
            raise ValueError('input timeouts exceed hardware operating bounds')
        if self.land_max_attempts > 10:
            raise ValueError('land_max_attempts must be <=10')
        if max(self.target_system_id, self.target_component_id, self.source_system_id) > 255 or self.source_component_id > 65535:
            raise ValueError('command ID out of range')
        if (self.target_system_id, self.target_component_id) == (self.source_system_id, self.source_component_id):
            raise ValueError('source and target IDs must differ')


def load_yaml(path):
    import yaml
    data = yaml.safe_load(open(path, encoding='utf-8'))['task_flight']['ros__parameters']
    data = dict(data)
    if data.pop('use_sim_time', False):
        raise ValueError('use_sim_time must be false')
    return Settings(**data)


def check_driver_limits(settings, data):
    """Validate the planned joint envelope against the live calibrated driver."""
    from math import radians
    if len(data) != 3 or not all(isfinite(x) for x in data):
        raise ValueError('J1 limits require finite [min_rad, max_rad, max_rate_rad_s]')
    low, high, rate = data
    if low >= high or rate <= 0:
        raise ValueError('Invalid J1 physical/rate limits')
    angles = (radians(settings.pose1_angle_deg), radians(settings.pose2_angle_deg))
    if min(angles) < low or max(angles) > high:
        raise ValueError('Planned J1 posture exceeds calibrated driver limits')
    if radians(settings.max_joint_rate_deg_s) > rate+1e-12:
        raise ValueError('Task max_joint_rate exceeds driver limit; lower task rate')
