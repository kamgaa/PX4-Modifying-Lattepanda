from dataclasses import replace
from math import radians, nan
import pytest
from palletrone_task_flight.configuration import Settings
from palletrone_task_flight.state_machine import TaskMachine, Position, Status, Joint


class Rig:
    def __init__(self,**kwargs):
        self.core=TaskMachine(Settings(dry_run=False,**kwargs))
        self.now=100.0;self.count=0
        self.xyz=(4.0,-3.0,2.0);self.yaw=0.7;self.theta=0.0
        self.armed=False;self.offboard=False;self.auto_land=False
        self.resets=(0,0,0,123);self.requests=[]

    def step(self,dt=0.05,pos=True,status=True,joint=True,follow=True,ready=True):
        self.now+=dt;self.count+=1
        target=self.core.last_target
        if follow and target is not None:
            self.xyz,self.yaw,self.theta=target.position,target.yaw,target.theta
        stamp=self.count*50000
        if pos:self.core.receive_position(self.now,Position(stamp,stamp,self.xyz,self.yaw,True,True,False,self.resets))
        if status:self.core.receive_status(self.now,Status(stamp,self.armed,not self.armed,self.offboard,self.auto_land))
        if joint:self.core.receive_joint(self.now,Joint(stamp,self.theta,ready))
        out=self.core.tick(self.now)
        if out.request_land:self.requests.append(self.now)
        return out

    def prepare(self):
        for _ in range(40):self.step()
        assert self.core.state=='READY',self.core.reason

    def start(self):
        self.prepare();self.armed=True;self.offboard=True;return self.step()

    def until(self,predicate,max_steps=15000):
        for _ in range(max_steps):
            out=self.step()
            if predicate(self.core):return out
        raise AssertionError((self.core.state,self.core.phase,self.core.reason))


@pytest.mark.parametrize('missing',['pos','status','joint'])
def test_missing_input_blocks_start(missing):
    r=Rig()
    for _ in range(80):assert r.step(**{missing:False}).target is None


def test_no_start_if_initially_armed():
    r=Rig();r.armed=r.offboard=True
    for _ in range(60):assert r.step().target is None
    r.armed=r.offboard=False;r.prepare()
    r.armed=r.offboard=True;r.step()
    assert r.core.state=='RUNNING'


@pytest.mark.parametrize('armed,offboard',[(True,False),(False,True),(False,False)])
def test_both_start_conditions(armed,offboard):
    r=Rig();r.prepare();r.armed=armed;r.offboard=offboard;r.step()
    assert r.core.state=='READY'


def test_early_arm_aborts():
    r=Rig();r.step();r.armed=r.offboard=True
    assert r.step().target is None
    assert r.core.state=='ABORTED'


def test_joint_calibration_and_ready_posture_required():
    r=Rig()
    for _ in range(40):assert r.step(ready=False).target is None
    r.theta=radians(10)
    for _ in range(40):assert r.step(follow=False).target is None


def test_fixed_capture_nonzero_origin():
    r=Rig();r.start()
    c=r.core.plan.contact
    assert r.core.plan.hover==(4.0,-3.0,1.5)
    for _ in range(100):r.step()
    assert r.core.plan.contact==c
    assert r.core.plan.start==(4.0,-3.0,2.0)


def test_full_mission_six_targets_then_land():
    r=Rig();r.start();observed=[];previous=None
    for _ in range(12000):
        r.step()
        phase=r.core.phase
        if phase!=previous and phase.startswith('AT_CONTACT_TARGET'):observed.append(phase)
        previous=phase
        if r.core.state=='LAND_REQUESTED':break
    assert len(observed)==6
    assert len(r.requests)==1
    assert r.core.last_target.theta==pytest.approx(0)
    assert r.core.last_target.position==pytest.approx(r.core.plan.hover)


@pytest.mark.parametrize('wrong',['position','yaw','joint'])
def test_actual_feedback_required_for_stage_advance(wrong):
    r=Rig();r.start();r.until(lambda c:c.phase.startswith('APPROACH'))
    index=r.core.index;elapsed=r.core.elapsed
    if wrong=='position':r.xyz=(r.xyz[0]+1,r.xyz[1],r.xyz[2])
    if wrong=='yaw':r.yaw+=radians(20)
    if wrong=='joint':r.theta+=radians(10)
    for _ in range(10):r.step(follow=False)
    assert r.core.index==index
    assert r.core.elapsed==elapsed


def test_joint_lag_pauses_then_latches_hold():
    r=Rig();r.start();r.until(lambda c:c.phase=='POSTURE_2')
    r.theta=radians(-10)
    out=None
    for _ in range(50):out=r.step(follow=False)
    assert r.core.state=='FAULT_HOLD'
    assert out.target is not None and out.joint_command is None
    for _ in range(30):r.step()
    assert r.core.state=='FAULT_HOLD' and not r.requests


def test_joint_timeout_holds_only_px4_commands():
    r=Rig();r.start()
    for _ in range(20):out=r.step(joint=False)
    assert r.core.state=='FAULT_HOLD'
    assert out.target is not None and out.joint_command is None
    r.step();assert r.core.state=='FAULT_HOLD'
    r.offboard=False;assert r.step().target is None


@pytest.mark.parametrize('fault',['pos','status','xy_reset','z_reset','heading_reset','reference','disarm','mode'])
def test_abort_is_irreversible(fault):
    r=Rig();r.start()
    if 'reset' in fault or fault=='reference':
        index={'xy_reset':0,'z_reset':1,'heading_reset':2,'reference':3}[fault]
        v=list(r.resets);v[index]+=1;r.resets=tuple(v)
    if fault=='disarm':r.armed=False
    if fault=='mode':r.offboard=False
    for _ in range(50):out=r.step(pos=fault!='pos',status=fault!='status')
    assert r.core.state=='ABORTED' and out.target is None
    r.armed=r.offboard=True
    for _ in range(50):assert r.step().target is None


def test_heading_gate_false_still_rejects_nan():
    r=Rig();r.start()
    r.core.receive_position(r.now,replace(r.core.position,stamp=r.core.position.stamp+1,yaw=nan))
    assert r.core.state=='ABORTED'


def test_repeated_joint_stamp_does_not_refresh():
    r=Rig();r.start();sample=r.core.joint
    for _ in range(20):
        r.core.receive_joint(r.now,sample);r.step(joint=False)
    assert r.core.state=='FAULT_HOLD'


@pytest.mark.parametrize('which',['position','status'])
def test_repeated_px4_stamp_does_not_refresh(which):
    r=Rig();r.start();sample=getattr(r.core,which)
    for _ in range(50):
        getattr(r.core,'receive_'+which)(r.now,sample)
        r.step(**{'pos' if which=='position' else 'status':False})
    assert r.core.state=='ABORTED'


def test_land_ack_is_not_mode_transition_and_retries_bounded():
    r=Rig();r.start();r.until(lambda c:c.state=='LAND_REQUESTED')
    for _ in range(150):r.core.receive_ack(0);r.step()
    assert r.core.state=='LAND_FAILED_HOLD' and len(r.requests)==3
    assert all(b-a>=2 for a,b in zip(r.requests,r.requests[1:]))
    r.offboard=False;r.auto_land=True
    assert r.step().target is None and r.core.state=='AUTO_LAND_HANDOVER'
    r.offboard=True;r.auto_land=False
    assert r.step().target is None


def test_land_denial_and_manual_takeover():
    r=Rig();r.start();r.until(lambda c:c.state=='LAND_REQUESTED')
    r.core.receive_ack(2);r.step()
    assert r.core.state=='LAND_REQUESTED'
    r.offboard=False
    for _ in range(150):assert r.step().target is None
    assert len(r.requests)==1


def test_land_blocked_when_joint_readiness_lost():
    r=Rig();r.start();r.until(lambda c:c.phase=='FINAL_HOVER')
    for _ in range(100):r.step(ready=False)
    assert r.core.state=='FAULT_HOLD' and not r.requests


def test_large_delay_aborts_small_delay_capped():
    r=Rig();r.start();r.step(dt=0.2)
    assert r.core.elapsed<=0.100001
    assert r.step(dt=0.31).target is None


def test_climb_timeout_is_not_mission_timeout():
    r=Rig();r.start()
    r.until(lambda c:r.now-c.mission_at>31)
    assert r.core.state=='RUNNING'
    r.core.mission_at=r.now-r.core.p.mission_timeout_s-1
    r.step();assert r.core.state=='FAULT_HOLD'


def test_dry_run_never_returns_commands():
    r=Rig();r.core=TaskMachine(Settings(dry_run=True));r.armed=r.offboard=True
    for _ in range(100):
        out=r.step()
        assert out.target is None and out.joint_command is None and not out.request_land
