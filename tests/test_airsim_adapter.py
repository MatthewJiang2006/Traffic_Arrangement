from math import pi,sqrt
from types import SimpleNamespace
import pytest
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig,AdapterFault
from logistics_park.common.messages import ControlCommand


class FakeRPC:
    def __init__(self):
        self.api=False;self.native=[];self.stamp=10;self.speed=0;self.fail=None
        self.pose={'position':{'x_val':2,'y_val':3,'z_val':4},'orientation':{'x_val':0,'y_val':0,'z_val':sqrt(.5),'w_val':sqrt(.5)}}
    def call(self,method,*args):
        if self.fail==method:raise OSError('RPC disconnected')
        if method=='ping':return True
        if method=='listVehicles':return ['truck']
        if method=='simIsPaused':return False
        if method=='isApiControlEnabled':return self.api
        if method=='enableApiControl':self.api=args[0];return
        if method=='setCarControls':
            self.native.append(args[0]);self.speed=0 if args[0]['brake']==1 else self.speed;return
        if method=='getCarState':return {'timestamp':self.stamp,'speed':self.speed,'kinematics_estimated':{'position':{'x_val':999,'y_val':999,'z_val':999}}}
        if method=='simGetObjectPose':return self.pose
        if method=='simGetCollisionInfo':return {'time_stamp':0,'has_collided':False,'object_name':''}
        raise AssertionError(method)
    def close(self):pass


@pytest.fixture
def fixture(tmp_path):
    rpc=FakeRPC();clock=SimpleNamespace(now=1.0)
    a=AirSimAdapter(AirSimConfig(('truck',),(10,20,30)),tmp_path/'adapter.jsonl',transport_factory=lambda:rpc,clock=lambda:clock.now,watchdog=False)
    a.open()
    yield a,rpc,clock
    a.close()


def command(clock,seq=0,**kwargs):
    return ControlCommand('truck',seq,clock.now,kwargs.get('v',1),kwargs.get('omega',.2),ttl=.8)


def test_world_pose_is_shared_frame_not_per_vehicle_spawn(fixture):
    a,rpc,clock=fixture;s=a.read_states()['truck']
    assert (s.x,s.y,a.telemetry['truck']['ue_z_m'])==(12,23,26)
    assert s.heading==pytest.approx(pi/2)
    clock.now+=.1;a.send(command(clock))
    assert rpc.native[-1]['steering']>0  # UE positive yaw is RIGHT.


def test_reverse_and_stationary_rotation_brake(fixture):
    a,rpc,clock=fixture
    with pytest.raises(AdapterFault):a.send(command(clock,v=0,omega=.5))
    assert rpc.native[-1]['brake']==1


def test_command_deadline_brakes_even_when_state_is_fresh(fixture):
    a,rpc,clock=fixture;a.send(command(clock));clock.now+=.9;rpc.stamp+=1;a.read_states();a.check_deadlines()
    assert 'command_timeout' in a.fault and rpc.native[-1]['brake']==1
    with pytest.raises(AdapterFault):a.send(command(clock,seq=1))


def test_missing_state_cannot_be_hidden_by_new_commands(fixture):
    a,rpc,clock=fixture;clock.now+=1.3
    with pytest.raises(AdapterFault):a.send(command(clock))
    assert 'state_timeout' in a.fault and rpc.native[-1]['brake']==1


def test_duplicate_or_expired_command_cannot_restore_motion(fixture):
    a,rpc,clock=fixture;a.send(command(clock))
    with pytest.raises(AdapterFault):a.send(command(clock))
    assert rpc.native[-1]['brake']==1


def test_frozen_source_timestamp_is_rejected(fixture):
    a,rpc,clock=fixture;clock.now+=1.3
    with pytest.raises(AdapterFault):a.read_states()
    assert 'source state frozen' in a.fault


def test_nonfinite_pose_and_failed_read_brake(fixture):
    a,rpc,clock=fixture;rpc.pose['position']['x_val']=float('nan')
    with pytest.raises(AdapterFault):a.read_states()
    assert rpc.native[-1]['brake']==1


def test_partial_rpc_failure_stops_owned_vehicle(fixture):
    a,rpc,clock=fixture;rpc.fail='getCarState'
    with pytest.raises(AdapterFault):a.read_states()
    assert rpc.native[-1]['brake']==1
    rpc.fail=None


def test_close_brakes_before_releasing_api(fixture):
    a,rpc,clock=fixture;rpc.speed=1;a.close()
    assert rpc.native[-1]['brake']==1 and not rpc.api
    assert a.close_result=={'stopped':True,'api_restored':True,'errors':[]}


def test_project_dispatch_connects_all_four_messages_before_read_fault(tmp_path):
    """Exercise the packaged application, not just the backend in isolation."""
    import json
    from pathlib import Path
    from logistics_park.map.map_loader import load_map
    from logistics_park.runtime.ue_dispatch import run_dispatch
    root=Path(__file__).resolve().parents[1]
    scenario=json.loads((root/'config/runtime/industrialarea_ue_four_truck.json').read_text())
    names=tuple(scenario['routes']);origin=tuple(scenario['adapter']['origin_ue_m'])
    class Fleet(FakeRPC):
        def __init__(self):super().__init__();self.apis={n:False for n in names};self.reads=0
        def call(self,method,*args):
            if method=='listVehicles':return list(names)
            if method=='isApiControlEnabled':return self.apis[args[0]]
            if method=='enableApiControl':self.apis[args[1]]=args[0];return
            if method=='simGetObjectPose':
                x,y=scenario['routes'][args[0]]['start']
                return {'position':{'x_val':x-origin[0],'y_val':y-origin[1],'z_val':origin[2]-.894},'orientation':self.pose['orientation']}
            if method=='getCarState':
                self.reads+=1
                if self.reads==13:raise OSError('forced read failure after first control frame')
                return {'timestamp':self.reads,'speed':0}
            return super().call(method,*args)
    class Observer:
        def read(self,name):
            x,y=scenario['routes'][name]['start']
            return {'bodies':[{'centre_ue_m':[x,y,2],'half_size_m':[3.667,1.29,1.5],'yaw_deg':90}]},[]
    rpc=Fleet();adapter=AirSimAdapter(AirSimConfig(names,origin),tmp_path/'adapter.jsonl',transport_factory=lambda:rpc,watchdog=False).open()
    result=run_dispatch(adapter,load_map(root/scenario['map']),json.loads((root/scenario['survey']).read_text()),scenario,Observer(),tmp_path)
    records=[json.loads(line) for line in (tmp_path/'adapter.jsonl').read_text().splitlines()]
    assert {r['vehicle'] for r in records if r['event']=='COMMAND'}==set(names)
    assert result['status']=='FAIL' and 'forced read failure' in result['failure'][0]
    assert adapter.closed and all(not enabled for enabled in rpc.apis.values())


def test_opt_in_reverse_uses_reverse_gear_and_signed_steering(tmp_path):
    rpc=FakeRPC();clock=SimpleNamespace(now=1.)
    a=AirSimAdapter(AirSimConfig(('truck',),(0,0,0),allow_reverse=True),tmp_path/'reverse.jsonl',transport_factory=lambda:rpc,clock=lambda:clock.now,watchdog=False).open()
    try:
        clock.now+=.1;a.send(command(clock,v=-1,omega=.2))
        assert rpc.native[-1]['manual_gear']==-1 and rpc.native[-1]['is_manual_gear']
        assert rpc.native[-1]['steering']<0
        rpc.speed=-.4;rpc.stamp+=1;clock.now+=.1;a.read_states()
        a.send(command(clock,seq=1,v=1,omega=0))
        assert rpc.native[-1]['brake']==1 and rpc.native[-1]['manual_gear']==-1
        rpc.stamp+=1;clock.now+=.1;a.read_states()
        a.send(command(clock,seq=2,v=1,omega=0))
        assert rpc.native[-1]['manual_gear']==1
    finally:a.close()


def test_reverse_remains_disabled_for_existing_scenarios(fixture):
    a,rpc,clock=fixture
    with pytest.raises(AdapterFault):a.send(command(clock,v=-1,omega=0))
    assert rpc.native[-1]['brake']==1


def test_overspeed_never_applies_throttle_and_brake_together(fixture):
    a,rpc,clock=fixture;rpc.speed=2;rpc.stamp+=1;a.read_states()
    clock.now+=.1;a.send(command(clock,v=1,omega=0))
    assert rpc.native[-1]['brake']>0 and rpc.native[-1]['throttle']==0
    clock.now+=.1;a.send(command(clock,seq=1,v=0,omega=0))
    assert rpc.native[-1]['brake']==1 and rpc.native[-1]['throttle']==0


def test_recovery_target_ramps_but_stop_is_immediate(fixture):
    from dataclasses import replace
    a,rpc,clock=fixture;a.config=replace(a.config,speed_rise_m_s2=.6)
    old=0.
    for seq in range(8):
        rpc.stamp+=1;a.read_states();clock.now+=.1
        a.send(command(clock,seq=seq,v=8,omega=0))
        target=a.applied_speed['truck']
        assert 0<target-old<=.06+1e-6
        old=target
    clock.now+=.1;a.send(command(clock,seq=8,v=0,omega=0))
    assert a.applied_speed['truck']==0
    assert rpc.native[-1]['brake']==1 and rpc.native[-1]['throttle']==0


def test_staging_watchdog_is_armed_once_and_still_times_out(fixture):
    a,rpc,clock=fixture
    clock.now=2;rpc.stamp+=1
    a.start_watchdog_after_staging()
    assert a.thread is not None and a.deadlines['truck']==pytest.approx(2.8)
    with pytest.raises(AdapterFault,match='Invalid'):a.start_watchdog_after_staging()
    a.send(command(clock));clock.now=3
    a.check_deadlines()
    assert a.fault=='command_timeout truck' and rpc.native[-1]['brake']==1


def test_staging_cannot_reset_a_deadline_after_driving(fixture):
    a,rpc,clock=fixture;a.send(command(clock))
    with pytest.raises(AdapterFault,match='Cannot rearm'):a.start_watchdog_after_staging()


def test_staging_rejects_an_observed_moving_truck(fixture):
    a,rpc,clock=fixture;rpc.speed=.5
    with pytest.raises(AdapterFault,match='Stationary'):a.start_watchdog_after_staging()
    assert rpc.native[-1]['brake']==1
