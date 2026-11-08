import math
import pytest
from pathlib import Path
from palletrone_task_flight.output import emit,command_topics
from palletrone_task_flight.state_machine import Output
from palletrone_task_flight.trajectory import Target


def test_dry_run_has_no_command_channels_or_publication():
    assert command_topics(True)==()
    output=Output(Target((1,2,3),0,0,0),0.0,True)
    emit(True,output,1,{},(1,1,245,191),1)  # Even a Land-bearing output cannot escape.


def test_position_only_fields_and_native_serialization():
    from rclpy.serialization import serialize_message,deserialize_message
    from palletrone_task_flight.messages import position_messages,check_publish_schemas
    check_publish_schemas()
    mode,goal=position_messages(12345,(4,-3,1.5),0.7)
    assert mode.position
    for name in mode.get_fields_and_field_types():
        if name not in ('timestamp','position'):assert getattr(mode,name) is False
    for name in ('velocity','acceleration','jerk'):
        assert all(math.isnan(v) for v in getattr(goal,name))
    assert math.isnan(goal.yawspeed)
    copy=deserialize_message(serialize_message(goal),type(goal))
    assert list(copy.position)==pytest.approx([4,-3,1.5])
    assert math.isnan(copy.yawspeed)


def test_only_land_command_emitted():
    from px4_msgs.msg import VehicleCommand
    from palletrone_task_flight.messages import land_command
    cmd=land_command(1,1,1,245,191,3)
    assert cmd.command==VehicleCommand.VEHICLE_CMD_NAV_LAND
    assert cmd.confirmation==2 and cmd.from_external
    assert math.isnan(cmd.param5) and math.isnan(cmd.param6) and math.isnan(cmd.param7)


def test_fault_hold_does_not_emit_joint_command():
    class Sink:
        def __init__(self):self.messages=[]
        def publish(self,m):self.messages.append(m)
    pubs={k:Sink() for k in ('mode','goal','joint','command')}
    emit(False,Output(Target((1,2,3),0,0,0),None,False),1,pubs,(1,1,245,191),0)
    assert len(pubs['mode'].messages)==len(pubs['goal'].messages)==1
    assert not pubs['joint'].messages and not pubs['command'].messages


def test_driver_default_launch_does_not_start_from_task_launch():
    root=Path(__file__).resolve().parents[1]
    launch=(root/'launch/task_flight.launch.py').read_text()
    assert 'dynamixel_fix_with_j1' not in launch
    import yaml
    p=yaml.safe_load((root/'config/j1_driver.yaml').read_text())['dynamixel_fix_with_j1']['ros__parameters']
    assert p['dry_run'] is True and p['calibration_confirmed'] is False
    driver=(root/'src/dynamixel_fix_with_j1.cpp').read_text()
    assert 'if(dry_ || startup_failed_) return;' in driver


def test_graph_guard_allows_support_but_rejects_competing_commands():
    # Real adapter method, no Node construction, DDS discovery or message substitutes.
    import importlib.util
    from types import SimpleNamespace
    from palletrone_task_flight.configuration import Settings
    from palletrone_task_flight.state_machine import TaskMachine
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('task_adapter',root/'scripts/task_flight.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    support=('/servo_angle','/servo_angle_cmd','/fmu/in/custom_control_mode','/j1_state','/j1_ready','/j1_limits')
    own=SimpleNamespace(node_name='task_flight',node_namespace='/')
    other=SimpleNamespace(node_name='support',node_namespace='/')
    graph={t:[own] for t in command_topics(False)}
    graph.update({t:[other] for t in support})
    adapter=SimpleNamespace(core=TaskMachine(Settings(dry_run=False)),get_name=lambda:'task_flight',
                            get_namespace=lambda:'/',get_publishers_info_by_topic=lambda t:graph.get(t,[]))
    module.TaskFlight.guard_graph(adapter)
    assert adapter.support_ready and adapter.core.state=='WAIT_INPUT'
    graph['/fmu/in/trajectory_setpoint'].append(other)
    module.TaskFlight.guard_graph(adapter)
    assert adapter.core.state=='ABORTED'
    graph['/fmu/in/trajectory_setpoint']=[own]
    graph['/control_mode_flags']=[other]
    adapter.core=TaskMachine(Settings(dry_run=False))
    module.TaskFlight.guard_graph(adapter)
    assert adapter.core.state=='ABORTED'


def test_ack_adapter_filters_address_but_never_confirms_land():
    import importlib.util
    from types import SimpleNamespace
    from px4_msgs.msg import VehicleCommand,VehicleCommandAck
    from palletrone_task_flight.configuration import Settings
    from palletrone_task_flight.state_machine import TaskMachine
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('ack_adapter',root/'scripts/task_flight.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    settings=Settings(dry_run=False);core=TaskMachine(settings);core.state='LAND_REQUESTED'
    adapter=SimpleNamespace(p=settings,core=core,last_ack_stamp=0,
                            get_logger=lambda:SimpleNamespace(info=lambda m:None))
    m=VehicleCommandAck();m.command=VehicleCommand.VEHICLE_CMD_NAV_LAND
    m.target_system=245;m.target_component=191;m.result=0
    for stamp,external in [(1,False),(2,True)]:
        m.timestamp=stamp;m.from_external=external
        module.TaskFlight.on_ack(adapter,m)
        assert core.last_ack==0 and core.state=='LAND_REQUESTED'
    m.timestamp=3;m.target_system=1;m.result=2
    module.TaskFlight.on_ack(adapter,m)
    assert core.last_ack==0 and adapter.last_ack_stamp==2
