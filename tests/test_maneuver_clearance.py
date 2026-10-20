"""Queues must stop outside the real W1 reverse entrance, not inside it."""
import math
from pathlib import Path
from types import SimpleNamespace
import pytest
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.runtime.transport_control import Execution,envelope
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic,sweeps_intersect
from logistics_park.scheduling.conflict_timing import maneuver_groups
from logistics_park.safety.collision import overlaps,separating_gap
from test_local_traffic import straight


@pytest.fixture
def entrance():
    root=Path(__file__).resolve().parents[1]
    network=ServiceNetwork(root/'config/topology/industrialarea_ue_internal_waiting_network.json')
    model=TransportMap(root/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    route=network.route('fleet24-start-15','W1-loading')
    plan=ServiceRouteGeometry(network,model).plan(SimpleNamespace(origin='fleet24-start-15',destination='W1-loading',route=route))
    group=next(g for g in maneuver_groups(plan) if any(model.edges[plan.edges[i]].get('reverse') for i in range(g[0],g[1]+1)))
    ex=Execution(plan,progress=group[2])
    count=math.ceil((group[3]-group[2])/.25)
    stations=[(group[2]+(group[3]-group[2])*i/count,envelope(VehicleState('loader',*plan.point(group[2]+(group[3]-group[2])*i/count),plan.heading(group[2]+(group[3]-group[2])*i/count)))) for i in range(count+1)]
    policy=LocalTraffic(point_conflicts=True)
    policy.passages['loader',group[0]]=dict(plan=plan,start=group[0],end=group[3],stations=stations,sweep=[b for _,b in stations],created_at=0)
    north=straight('north',41.1,100,math.pi/2)
    states={'loader':VehicleState('loader',*plan.point(ex.progress),plan.heading(ex.progress)), 'north':VehicleState('north',41.1,100,math.pi/2)}
    return policy,{'loader':ex,'north':north},states,group


def test_opposite_queue_stops_before_w1_reverse_swept_body(entrance):
    policy,ex,states,group=entrance
    policy._protect_maneuver_space(states,ex,envelope)
    hold=policy.maneuver_holds['north'];assert hold['blocked_by']=='loader'
    plan=ex['north'].plan;stop=hold['stop_s']
    sweep=policy.passages['loader',group[0]]['sweep']
    # The previous intersection hold put the northbound reference at y~121.
    # Its body intersects the reverse sweep. The new holding pose clears it.
    assert sweeps_intersect([envelope(VehicleState('north',41.1,121,math.pi/2))],sweep,.35)
    assert not sweeps_intersect([envelope(VehicleState('north',*plan.point(stop),plan.heading(stop)))],sweep,.35)
    forecast,_=policy._forecast('north',states['north'],ex['north'],8,None,envelope)
    assert max(b.y for b in forecast)<=100+stop+.75+1e-6


def test_parallel_road_outside_reverse_entrance_keeps_moving(entrance):
    policy,ex,states,_=entrance
    ex['north']=straight('north',55,100,math.pi/2);states['north'].x=55
    policy._protect_maneuver_space(states,ex,envelope)
    assert 'north' not in policy.maneuver_holds


def test_reverse_clearance_releases_when_owner_has_completed_the_maneuver(entrance):
    policy,ex,states,group=entrance
    policy._protect_maneuver_space(states,ex,envelope);assert 'north' in policy.maneuver_holds
    ex['loader'].progress=group[3]+1
    policy._protect_maneuver_space(states,ex,envelope)
    assert not policy.maneuver_holds


def test_submillimetre_w1_attachment_cannot_rotate_the_predicted_truck(entrance):
    _,ex,_,_=entrance;plan=ex['loader'].plan
    i=next(i for i in range(len(plan.edges)) if plan.edge_ends[i]-plan.edge_starts[i]<.001)
    station=(plan.edge_starts[i]+plan.edge_ends[i])/2
    delta=(plan.heading(station)-plan.heading(plan.edge_starts[i]-.02)+math.pi)%(2*math.pi)-math.pi
    assert abs(delta)<.01


def test_following_maneuver_owner_cannot_hold_its_physical_lane_leader(entrance):
    policy,ex,states,_=entrance;plan=ex['loader'].plan
    # The leader is on the same directed approach, before the loader's gear
    # change. Its road continues past the loader's planned reverse entrance.
    progress=plan.checkpoints[0]['s']-4
    point=plan.point(progress);heading=plan.heading(progress)
    ex['north']=straight('north',*point,heading);states['north']=VehicleState('north',*point,heading)
    policy._protect_maneuver_space(states,ex,envelope)
    assert 'north' not in policy.maneuver_holds


def test_later_reverse_booking_yields_to_an_older_committed_passage(entrance):
    policy,ex,states,group=entrance
    north=ex['north'];north.plan.edge_starts=[0.];north.plan.edge_ends=[40.]
    policy.passages['loader',group[0]]['created_at']=10
    policy.passages['north',0]=dict(plan=north.plan,start=0,end=40,sweep=[],stations=[],created_at=0)
    policy._protect_maneuver_space(states,ex,envelope)
    assert 'north' not in policy.maneuver_holds


def test_older_booking_cannot_override_a_reverse_maneuver_already_in_progress(entrance):
    policy,ex,states,group=entrance
    loader=ex['loader'];loader.progress=loader.plan.checkpoints[0]['s']+1
    north=ex['north'];north.plan.edge_starts=[0.];north.plan.edge_ends=[40.]
    policy.passages['loader',group[0]]['created_at']=10
    policy.passages['north',0]=dict(plan=north.plan,start=0,end=40,sweep=[],stations=[],created_at=0)
    policy._protect_maneuver_space(states,ex,envelope)
    assert policy.maneuver_holds['north']['blocked_by']=='loader'


def test_yielding_owner_cannot_stop_the_vehicle_it_has_allowed_to_pass(entrance):
    policy,ex,states,_=entrance
    policy.paused_by['loader']={('north',0)}
    policy._protect_maneuver_space(states,ex,envelope)
    assert 'north' not in policy.maneuver_holds


def test_gear_restart_cannot_override_a_destination_stop(entrance):
    from logistics_park.runtime.transport_control import TransportController
    _,ex,_,_=entrance;plan=ex['loader'].plan;gear=plan.checkpoints[0]['s']
    controller=TransportController(plan.model,{'truck':plan},collect_at_destination=True)
    controller.activate('truck',0)
    execution=controller.executions['truck'];execution.progress=gear+.01;execution.checkpoint=1
    controller.destination_holds['truck']=gear+.03
    state=VehicleState('truck',*plan.point(gear+.01),plan.heading(gear+.01),speed=0)
    assert controller.tick({'truck':state},1)['truck'][0]==0


def test_congested_road_detour_to_waiting_bay_does_not_use_w1_loading_as_a_shortcut():
    root=Path(__file__).resolve().parents[1]
    network=ServiceNetwork(root/'config/topology/industrialarea_ue_internal_waiting_network.json')
    incoming='road_j_1_1_j_1_2__p0__service_0'
    costs={k:30 for k,e in network.edges.items() if e['capacity_alias'] in ('road_j_1_1_j_1_2','road_j_1_2_j_0_2')}
    unrestricted=network.route(network.edges[incoming]['to_node'],'Q9',incoming_edge=incoming,
                               edge_costs=costs,transit_warehouses={'W1'})
    assert 'access_002' in unrestricted['edge_ids']  # Actual round25 shortcut.
    road=network.route(network.edges[incoming]['to_node'],'Q9',incoming_edge=incoming,
                      edge_costs=costs,transit_warehouses=frozenset())
    assert road and 'access_002' not in road['edge_ids']
    assert all(network.services.get(network.edges[k]['to_node'],{}).get('kind') not in ('loading','unloading') for k in road['edge_ids'])


def test_physical_leader_beyond_the_planned_gear_endpoint_must_clear_first():
    root=Path(__file__).resolve().parents[1]
    network=ServiceNetwork(root/'config/topology/industrialarea_ue_internal_waiting_network.json')
    model=TransportMap(root/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    route=network.route('fleet24-start-06','Q29')
    plan=ServiceRouteGeometry(network,model).plan(SimpleNamespace(origin='fleet24-start-06',destination='Q29',route=route))
    group=next(g for g in maneuver_groups(plan) if any(model.edges[plan.edges[i]].get('reverse') for i in range(g[0],g[1]+1)))
    progress=47.64813556764819
    loader=Execution(plan,progress=progress)
    states={'loader':VehicleState('loader',*plan.point(progress),plan.heading(progress)),
            'front':VehicleState('front',5.1594,266.9308,0)}
    executions={'loader':loader,'front':straight('front',5.1594,266.9308)}
    policy=LocalTraffic(point_conflicts=True)
    count=math.ceil((group[3]-group[2])/.25)
    stations=[(group[2]+(group[3]-group[2])*i/count,envelope(VehicleState('loader',*plan.point(group[2]+(group[3]-group[2])*i/count),plan.heading(group[2]+(group[3]-group[2])*i/count)))) for i in range(count+1)]
    policy.passages['loader',group[0]]=dict(plan=plan,start=group[0],end=group[3],stations=stations,sweep=[b for _,b in stations],created_at=0)
    error,_=plan.project((states['front'].x,states['front'].y),progress,behind=0,ahead=40)
    assert error>7  # The front is beyond the planned forward gear endpoint.
    # Its stopped body really intersects the planned reverse entrance; the
    # safe resolution is to let it continue forward first, rather than hold it.
    reverse=[b for station,b in stations if plan.reverse(station)]
    assert sweeps_intersect([envelope(states['front'])],reverse,.35)
    policy._protect_maneuver_space(states,executions,envelope)
    assert 'front' not in policy.maneuver_holds


def north_pair():
    """Exact directed routes involved in round28's opposing-lane deadlock."""
    root=Path(__file__).resolve().parents[1]
    network=ServiceNetwork(root/'config/topology/industrialarea_ue_internal_waiting_network.json')
    model=TransportMap(root/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    geometry=ServiceRouteGeometry(network,model)
    plans={}
    for name,origin,destination in [('exit','fleet24-start-04','Q4'),('reverse','fleet24-start-19','Q32')]:
        plans[name]=geometry.plan(SimpleNamespace(origin=origin,destination=destination,route=network.route(origin,destination)))
    executions={name:Execution(plan,progress=(140.64 if name=='exit' else 264.0)) for name,plan in plans.items()}
    states={name:VehicleState(name,*ex.plan.point(ex.progress),ex.plan.heading(ex.progress)) for name,ex in executions.items()}
    policy=LocalTraffic(point_conflicts=True)
    for name,start,end,created in [('exit',29,32,70.),('reverse',71,76,81.)]:
        plan=plans[name];entry=executions[name].progress;limit=min(plan.length,plan.edge_ends[end]+11.344)
        count=math.ceil((limit-entry)/.25)
        stations=[(entry+(limit-entry)*i/count,envelope(VehicleState(name,*plan.point(entry+(limit-entry)*i/count),plan.heading(entry+(limit-entry)*i/count)))) for i in range(count+1)]
        policy.passages[name,start]=dict(plan=plan,start=start,end=plan.edge_ends[end],created_at=created,stations=stations,sweep=[b for _,b in stations])
    return policy,executions,states


def test_earlier_exit_survives_release_of_its_old_junction_permission():
    policy,ex,states=north_pair()
    policy.prepare(states,ex,{},envelope,82.)
    assert 'exit' not in policy.maneuver_holds
    assert policy.clearance_pauses['reverse']=={'exit'}
    ex['exit'].progress=146.65
    states['exit']=VehicleState('exit',*ex['exit'].plan.point(146.65),ex['exit'].plan.heading(146.65))
    policy.prepare(states,ex,{},envelope,84.)
    assert ('exit',29) not in policy.passages
    assert 'exit' not in policy.maneuver_holds
    result=policy.regulate(states,ex,{'exit':(8,0),'reverse':(1,0)},{},84.,envelope)
    assert result['exit'][0]>0 and result['reverse'][0]==0
    right=policy.clearance_rights['reverse',71,'exit']
    ex['exit'].progress=right['end']+.5
    states['exit']=VehicleState('exit',*ex['exit'].plan.point(ex['exit'].progress),ex['exit'].plan.heading(ex['exit'].progress))
    policy.prepare(states,ex,{},envelope,90.)
    assert not policy.clearance_pauses and not policy.clearance_rights


def test_road_truck_already_inside_reverse_hold_can_clear_on_its_legal_road():
    policy,ex,states=north_pair()
    del policy.passages['exit',29]
    ex['exit'].progress=146.65;ex['reverse'].progress=281.92;ex['reverse'].checkpoint=1
    states={name:VehicleState(name,*e.plan.point(e.progress),e.plan.heading(e.progress)) for name,e in ex.items()}
    policy.prepare(states,ex,{},envelope,100.)
    assert 'exit' not in policy.maneuver_holds and policy.clearance_pauses['reverse']=={'exit'}
    result=policy.regulate(states,ex,{'exit':(8,0),'reverse':(-1,0)},{},100.,envelope)
    assert result['exit'][0]>0 and result['reverse'][0]==0


def test_clearance_cannot_grant_a_forward_exit_that_stops_inside_the_reverse_space():
    policy,ex,states=north_pair()
    del policy.passages['exit',29]
    ex['exit'].progress=146.65
    states['exit']=VehicleState('exit',*ex['exit'].plan.point(146.65),ex['exit'].plan.heading(146.65))
    policy.prepare(states,ex,{'exit':147.},envelope,84.)
    assert not policy.clearance_rights and not policy.clearance_pauses
    assert policy.maneuver_holds['exit']['blocked_by']=='reverse'


def test_clearance_duty_is_refused_when_the_grantee_itself_is_walled_in():
    """round38: the paused reverse body blocks the forward truck's whole path.

    Granting the duty then pauses the reversing truck for a forward truck that
    cannot move either, producing reciprocal waiting. The duty must be refused
    so the committed reverse action keeps its precedence.
    """
    policy,ex,states=north_pair()
    policy._protect_maneuver_space(states,ex,envelope)
    right=policy.clearance_rights['reverse',71,'exit']
    assert policy._clearance_executable('reverse','exit',ex['exit'],states,envelope,right['end'])
    # The northbound body now sits in the exit's stop span, 35 cm short of the
    # coarse hard-limit check but fully inside the regulator's margin.
    blocked=dict(states)
    blocked['reverse']=VehicleState('reverse',-31.5,263.13,0.)
    assert not overlaps(envelope(blocked['exit']),envelope(blocked['reverse']),.2)
    assert not policy._clearance_executable('reverse','exit',ex['exit'],blocked,envelope,right['end'])
    fresh=LocalTraffic(point_conflicts=True);fresh.passages=policy.passages
    fresh._protect_maneuver_space(blocked,ex,envelope)
    assert not fresh.clearance_rights and not fresh.clearance_pauses
    assert any(e['event']=='CLEARANCE_PRIORITY_REJECTED' and e['reason']=='unexecutable_clearance'
               for e in fresh.events)


def test_clearance_rejects_end_after_an_unapproved_maneuver_stop_line():
    policy,ex,states=north_pair()
    road=ex['exit'];road.progress=146.65
    states['exit']=VehicleState('exit',*road.plan.point(road.progress),road.plan.heading(road.progress))
    policy.action_groups={'exit':[(99,99,155.,160.)]}
    assert not policy._grant_clearance('reverse',71,'exit',road,states,[147.,147.5,148.,148.5],0,envelope)
    assert not policy.clearance_rights and not policy.clearance_pauses
    assert policy.events[-1]['reason']=='ungranted_stop_line'
    assert policy.events[-1]['stop_s']==pytest.approx(147.2)


def test_approved_next_maneuver_does_not_reject_an_otherwise_valid_clearance():
    policy,ex,states=north_pair()
    policy._executions=ex
    road=ex['exit'];road.progress=146.65
    states['exit']=VehicleState('exit',*road.plan.point(road.progress),road.plan.heading(road.progress))
    policy.action_groups={'exit':[(99,99,155.,160.)]}
    policy.passages['exit',99]=dict(plan=road.plan,end=160.,sweep=[])
    assert policy._grant_clearance('reverse',71,'exit',road,states,[147.,147.5,148.,148.5],0,envelope)


def test_changed_stop_line_withdraws_clearance_duty_without_revoking_lease():
    policy,ex,states=north_pair()
    policy.prepare(states,ex,{},envelope,82.)
    right=policy.clearance_rights['reverse',71,'exit'];owner_lease=policy.passages['reverse',71]
    policy.action_groups={'exit':[(99,99,right['end']+7.7,right['end']+12.)]}
    policy.prepare(states,ex,{},envelope,83.)
    assert not policy.clearance_rights and not policy.clearance_pauses
    assert policy.passages['reverse',71] is owner_lease
    assert any(e['event']=='CLEARANCE_PRIORITY_WITHDRAWN' and not e['physical_leases_revoked'] for e in policy.events)


def round38_pair(tmp_path):
    """Exact round38 trucks, rebuilt from the frozen live diagnostic frame."""
    import json
    root=Path(__file__).resolve().parents[1]
    raw=json.loads((root/'tests/fixtures/round38_clearance_deadlock.json').read_text())
    spec=json.loads((root/'config/runtime/industrialarea_ue_fleet16_flow_optimized.json').read_text())
    # The recorded frame predates C's 1.6 m pavement correction. Reconstruct
    # that historical bend in temporary test inputs; keep the live map fixed.
    def historical(kind, source):
        data=json.loads((root/source).read_text())
        def gate_point(point):
            x,y=point
            if abs(y-297.4)<1e-7:y=299.
            elif abs(y-301.11335)<1e-7:y=301.91335
            return [x,y]
        for edge in data['edges']:
            key=edge['edge_id']
            if key.startswith('uturn_C'):
                edge['centerline']=[[x,y+1.6] for x,y in edge['centerline']]
            elif 'gate_north_0' in key:
                edge['centerline']=[gate_point(p) for p in edge['centerline']]
            else:continue
            old_length=edge.get('length_m')
            length=sum(math.dist(a,b) for a,b in zip(edge['centerline'],edge['centerline'][1:]))
            if old_length:
                edge['length_m']=length
                if 'estimated_travel_time_s' in edge:edge['estimated_travel_time_s']*=length/old_length
        for node in data['nodes']:
            if node['node_id'].startswith('uturn_C') and node['node_id']!='uturn_C_shift_end':
                node['position'][1]+=1.6
            elif 'gate_north_0' in node['node_id']:
                node['position']=gate_point(node['position'])
        if kind=='transport':
            data['physical_centerlines']['uturn_C']=[[x,y+1.6] for x,y in data['physical_centerlines']['uturn_C']]
        path=tmp_path/(kind+'.json');path.write_text(json.dumps(data));return path
    network=ServiceNetwork(historical('network',spec['network']),blocked_edges=spec['blocked_candidate_edges'])
    model=TransportMap(historical('transport',spec['map']));geometry=ServiceRouteGeometry(network,model)
    executions={};states={}
    for name,row in raw['vehicles'].items():
        keys=row['route_keys']
        origin=network.edges[keys[0]]['from_node']
        plan=geometry.plan(SimpleNamespace(origin=origin,destination=row['destination'],
            route=dict(edge_ids=keys)))
        executions[name]=Execution(plan,progress=row['progress_m'])
        executions[name].checkpoint=row['checkpoint_index']
        states[name]=VehicleState(name,*row['position'][:3],row['position'][3])
    policy=LocalTraffic(point_conflicts=True)
    policy.action_groups={n:maneuver_groups(ex.plan) for n,ex in executions.items()}
    for name,row in raw['vehicles'].items():
        for lease in row['leases']:
            plan=executions[name].plan;lo=max(0.,math.floor(executions[name].progress*4)/4)
            hi=min(plan.length,lease['end_m']+11.344);count=max(1,math.ceil((hi-lo)/.25))
            stations=[(lo+(hi-lo)*i/count,envelope(VehicleState(name,*plan.point(lo+(hi-lo)*i/count),
                        plan.heading(lo+(hi-lo)*i/count)))) for i in range(count+1)]
            policy.passages[name,lease['start']]=dict(plan=plan,start=lease['start'],end=lease['end_m'],
                created_at=raw['frame_t_s']-lease['age_s'],stations=stations,sweep=[b for _,b in stations])
    return policy,executions,states,raw


def test_round38_unexecutable_clearance_duty_is_refused_and_reverse_keeps_precedence(tmp_path):
    """round38: pausing the reversing truck for a walled-in forward truck deadlocks."""
    policy,ex,states,raw=round38_pair(tmp_path)
    owner='Car_1_1';grantee='Car_0_1'
    # The reconstruction matches the saved measured poses to millimetres.
    assert math.dist(ex[grantee].plan.point(ex[grantee].progress)[:2],
                     (states[grantee].x,states[grantee].y))<.05
    assert ex[owner].plan.reverse(ex[owner].progress)
    gap=raw['measured_gaps_m']
    assert abs(separating_gap(envelope(states[grantee]),envelope(states[owner]),.35)
               -gap['margin_0_35'])<.01
    end=raw['clearance_right']['end_m']
    policy.prepare(states,ex,{},envelope,raw['frame_t_s'])
    assert not policy.clearance_rights and not policy.clearance_pauses
    assert policy.maneuver_holds[grantee]['blocked_by']==owner
    assert any(e['event']=='CLEARANCE_PRIORITY_REJECTED' and e['reason']=='unexecutable_clearance'
               for e in policy.events)
    result=policy.regulate(states,ex,{grantee:(8.,0.),owner:(-1.,0.)},{},raw['frame_t_s'],envelope)
    assert result[owner][0]<0


def test_stopped_truck_with_settling_buffer_can_request_clearance(entrance,monkeypatch):
    policy,ex,states,group=entrance
    policy._protect_maneuver_space(states,ex,envelope)
    stop=policy.maneuver_holds['north']['stop_s']
    ex['north'].progress=stop-.283
    states['north']=VehicleState('north',*ex['north'].plan.point(ex['north'].progress),math.pi/2)
    attempts=[]
    def checked_grant(*args):
        attempts.append(args[:3]);return True
    monkeypatch.setattr(policy,'_grant_clearance',checked_grant)
    policy._protect_maneuver_space(states,ex,envelope)
    assert attempts and 'north' not in policy.maneuver_holds


def test_clearance_allows_only_approved_forward_service_exit():
    ex=straight('exit',0,0);p=ex.plan;key=p.edges[0]
    p.model.edges[key]['kind']='service_access';p.edge_starts=[0.];p.edge_ends=[100.]
    policy=LocalTraffic(point_conflicts=True)
    states={'exit':VehicleState('exit',0,0,0),'owner':VehicleState('owner',50,50,0)}
    assert not policy._clearance_executable('owner','exit',ex,states,envelope,6.)
    policy.passages['exit',0]=dict(plan=p,end=10.)
    assert policy._clearance_executable('owner','exit',ex,states,envelope,6.)
    assert not policy._clearance_executable('owner','exit',ex,states,envelope,12.)
    p.reverse=lambda s:True
    assert not policy._clearance_executable('owner','exit',ex,states,envelope,6.)
    p.reverse=lambda s:False
    states['owner']=VehicleState('owner',6,0,0)
    assert not policy._clearance_executable('owner','exit',ex,states,envelope,6.)


def test_third_stopped_body_prevents_and_withdraws_clearance_duty():
    policy,ex,states=north_pair()
    policy.compact_permissions=True
    policy._protect_maneuver_space(states,ex,envelope)
    right=policy.clearance_rights['reverse',71,'exit']
    other_s=min(right['end'],ex['exit'].progress+3.)
    states['blocker']=VehicleState('blocker',*ex['exit'].plan.point(other_s),ex['exit'].plan.heading(other_s))
    assert not policy._clearance_executable('reverse','exit',ex['exit'],states,envelope,right['end'])
    policy._protect_maneuver_space(states,ex,envelope)
    assert ('reverse',71,'exit') not in policy.clearance_rights
    assert 'reverse' not in policy.clearance_pauses
    assert any(e['event']=='CLEARANCE_PRIORITY_WITHDRAWN' and e['reason']=='clearance_no_longer_executable' for e in policy.events)
