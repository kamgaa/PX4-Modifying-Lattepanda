import math
import pytest
from palletrone_task_flight.geometry import Geometry, norm, sub, rotate, direction
from palletrone_task_flight.configuration import Settings
from palletrone_task_flight.trajectory import Plan


def test_rod_and_contact_geometry():
    g=Geometry(reference_to_bottom=(0.03,-0.02,0.08))
    c=(3.0,-2.0,1.0)
    for yaw in (0.0,0.7,3.2,-2.8):
        for theta in (0.0,math.pi/6,-0.2):
            for d in (0.0,0.15,0.30):
                p=g.target(c,yaw,theta,d)
                m,e=g.points(p,yaw,theta)
                assert norm(sub(e,m))==pytest.approx(0.6)
                assert norm(sub(c,e))==pytest.approx(d)
                unit=rotate(yaw,direction(theta))
                assert sub(c,m)==pytest.approx(tuple((0.6+d)*v for v in unit))
                if d==0: assert e==pytest.approx(c)


def test_thirty_degree_approach():
    g=Geometry();c=(1,2,3);theta=math.pi/6
    delta=sub(g.target(c,0,theta,0),g.target(c,0,theta,0.3))
    assert delta==pytest.approx((0.259807621,0,0.15))


def test_posture_change_and_mount():
    g=Geometry();c=(0,0,0)
    delta=sub(g.target(c,0,math.pi/6,0.3),g.target(c,0,0,0.3))
    assert delta==pytest.approx((0.120577137,0,-0.45))
    g2=Geometry(reference_to_bottom=(0.1,0.2,0.3))
    p=g2.target(c,math.pi/2,0,0.3)
    assert p==pytest.approx((0.2,-0.89,-0.3))


def test_default_sequence_and_max_rise():
    plan=Plan((4,-3,2),0.7,Settings())
    approaches=[s.name for s in plan.segments if s.name.startswith('APPROACH')]
    assert approaches==['APPROACH_P1_0','APPROACH_P1_30','APPROACH_P1_60',
                        'APPROACH_P2_60','APPROACH_P2_30','APPROACH_P2_0']
    assert plan.segments[-1].end.position==pytest.approx(plan.hover)
    assert plan.summary()['max_rise_from_start_m']==pytest.approx(0.95)


def test_nonreversed_sequence_returns_to_hover():
    plan=Plan((0,0,0),0,Settings(reverse_second_pass=False))
    assert [s.name for s in plan.segments if s.name.startswith('APPROACH_P2')]==[
        'APPROACH_P2_0','APPROACH_P2_30','APPROACH_P2_60']
    assert plan.segments[-1].end.position==pytest.approx(plan.hover)


def test_all_segments_continuous_and_rate_limited():
    p=Settings(orbit_duration_s=0.1,posture_change_duration_s=0.1,approach_duration_s=0.1,retract_duration_s=0.1)
    plan=Plan((2,3,4),3.1,p)
    for a,b in zip(plan.segments,plan.segments[1:]):
        assert a.sample(a.duration).position==pytest.approx(b.sample(0).position)
        assert a.sample(a.duration).theta==pytest.approx(b.sample(0).theta)
        assert a.sample(a.duration).yaw==pytest.approx(b.sample(0).yaw)
    for segment in plan.segments:
        dt=segment.duration/1000
        old=segment.sample(0)
        for i in range(1,1001):
            target=segment.sample(i*dt)
            assert norm(sub(target.position,old.position))/dt<=p.max_reference_speed_mps+1e-6
            assert abs(target.yaw-old.yaw)/dt<=math.radians(p.max_reference_yaw_rate_deg_s)+1e-6
            assert abs(target.theta-old.theta)/dt<=math.radians(p.max_joint_rate_deg_s)+1e-6
            if segment.kind!='climb':
                m,e=plan.geometry.points(target.position,target.yaw,target.theta)
                assert norm(sub(plan.contact,e))==pytest.approx(target.distance)
            old=target


@pytest.mark.parametrize('kwargs',[{'rod_length_m':0},{'pose2_angle_deg':float('nan')},
    {'px4_reference_to_bottom_frd_m':[0,0]},{'dry_run':'false'},{'land_max_attempts':2.5},
    {'position_timeout_s':10},{'pose1_angle_deg':10},{'azimuth_offsets_deg':[0,0,60]}])
def test_bad_parameters(kwargs):
    with pytest.raises(ValueError): Settings(**kwargs)


def test_zero_angles_offsets_allowed():
    Settings(pose2_angle_deg=0,px4_reference_to_bottom_frd_m=[0,0,0],j1_mount_from_bottom_frd_m=[0,0,0])


def test_contact_endpoint_roundoff_cannot_overshoot():
    from palletrone_task_flight.geometry import smoothstep
    plan=Plan((4,-3,2),0.7,Settings())
    for segment in plan.segments:
        for delta in (0.0,1e-15,1e-12,1e-9,1e-7):
            u=1.0-delta
            assert 0.0 <= smoothstep(u) <= 1.0
            target=segment.sample(segment.duration*u)
            assert 0.0 <= target.distance <= plan.p.standoff_distance_m


def test_driver_limits_block_incompatible_plan_before_start():
    from palletrone_task_flight.configuration import check_driver_limits
    check_driver_limits(Settings(),[math.radians(-5),math.radians(35),math.radians(15)])
    for values in ([0,0,1],[0,1,float('nan')],[0,math.radians(20),1],[-1,1,0.01],[0,1]):
        with pytest.raises(ValueError): check_driver_limits(Settings(),values)
