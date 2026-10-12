#!/usr/bin/env python3
"""Add independent, forward-through truck parking outside both wide-road ends.

Candidate access remains disabled until matching UE entry/exit evidence exists.
"""
import copy
import hashlib
import json
import math
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from ue_uturn_paths import arc, line
from logistics_park.safety.collision import OBB, overlaps

D = 1.16425
PLAN = ROOT/'config/planning/ue_parking_expansion.json'


def join(*parts):
    result = []
    for part in parts:
        if result and math.dist(result[-1], part[0]) > 1e-6:
            raise ValueError('disconnected parking path')
        result.extend(part[1:] if result else part)
    return result


def paths(x, north=False):
    # South: road outbound S -> west entrance aisle -> N bay -> east exit aisle
    # -> road inbound N. North reflects this geometry and uses the correct lanes.
    out_x, in_x = (-94.0937, -86.4937) if north else (-86.4936, -97.8936)
    gate = 49.3973
    entry = join(line([out_x, gate], [out_x, 14]),
                 arc([out_x-4, 14], [out_x, 14], [out_x-4, 10], -1, 4),
                 line([out_x-4, 10], [x+4, 10]),
                 arc([x+4, 14], [x+4, 10], [x, 14], -1, 4),
                 line([x, 14], [x, 26-D]))
    exit_path = join(line([x, 26-D], [x, 36]),
                     arc([x+4, 36], [x, 36], [x+4, 40], -1, 4),
                     line([x+4, 40], [in_x-4, 40]),
                     arc([in_x-4, 44], [in_x-4, 40], [in_x, 44], 1, 4),
                     line([in_x, 44], [in_x, gate]))
    if north:
        mirror = 304.8267+gate
        entry = [[px, mirror-py] for px, py in entry]
        exit_path = [[px, mirror-py] for px, py in exit_path]
    return entry, exit_path


def build():
    sites=[]
    for north, count in ((False, 7), (True, 6)):
        for i in range(count):
            key=f'Q{13+len(sites)}'; x=-110-4*i
            entry, exit_path=paths(x, north)
            heading=270 if north else 90
            rear=entry[-1]; center=[rear[0], rear[1]+(-D if north else D)]
            sites.append(dict(id=key,kind='waiting',capacity=1,xy_m=center,
                rear_xy_m=rear,heading_deg=heading,parking_envelope_m=[9,3.3],
                region='north_outer' if north else 'south_outer',
                entry_gate='gate_north_0_out' if north else 'gate_south_0_out_right',
                exit_gate='gate_north_0_in_right' if north else 'gate_south_0_in_right',
                entry_rear_path=entry,exit_rear_path=exit_path,
                entry_mode='forward',exit_mode='forward',operationally_enabled=False))
    # Parked neighbours must not obstruct any entry/exit, even when all slots fill.
    for a in sites:
        for mode in ('entry_rear_path','exit_rear_path'):
            for p,q in zip(a[mode],a[mode][1:]):
                h=math.atan2(q[1]-p[1],q[0]-p[0])
                body=OBB(p[0]+(D+.75)*math.cos(h),p[1]+(D+.75)*math.sin(h),h,8.344,2.6)
                for b in sites:
                    if a['id']==b['id']:continue
                    hb=math.radians(b['heading_deg']);bx,by=b['xy_m']
                    if overlaps(body,OBB(bx+.75*math.cos(hb),by+.75*math.sin(hb),hb,8.344,2.6),.35):
                        raise ValueError(f"{a['id']} {mode} blocked by parked {b['id']}")
    source=ROOT/'config/topology/industrialarea_ue_fleet16_network.json'
    old=json.loads(source.read_text())
    existing=[k for k,v in old['service_nodes'].items() if v['kind']=='waiting']
    plan=dict(status='PENDING_UE_ACCESS_VALIDATION',coordinate_frame='UE absolute world metres',
        target_capacity=24,existing_waiting_nodes=existing,retired_waiting_nodes=['Q3'],
        added_capacity=len(sites),planned_capacity=len(existing)+len(sites),
        source_network=str(source.relative_to(ROOT)),source_network_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        map_sha256=old['map_sha256'],sites=sites,
        layout='Two one-way outer parking courts; separate entrance/exit aisles; forward-through bays',
        neighbouring_occupied_bay_clearance='PASS at 0.35m full-body margins',camera_changed=False)
    PLAN.write_text(json.dumps(plan,indent=2)+'\n')
    print('planned',plan['planned_capacity'],'slots;',len(sites),'new independent entries/exits')



def compile_access():
    from build_ue_waiting_scan import build as build_scan
    plan=json.loads(PLAN.read_text())
    probe_path=ROOT/'runs/ue_scene/station_siting/parking_expansion_probe.json'
    probe=json.loads(probe_path.read_text())
    patch=json.loads((ROOT/'runs/ue_scene/parking_expansion/map_patch.json').read_text())
    if probe['status']!='PASS' or patch['status']!='PASS' or patch['changed_existing_actors']:
        raise ValueError('Static clearance and preservation must pass before compiling')
    network=json.loads((ROOT/plan['source_network']).read_text())
    maps={n:hashlib.sha256((Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')/n).read_bytes()).hexdigest() for n in plan['map_sha256']}
    if probe.get('map_sha256')!=maps or probe.get('sites_sha256')!=hashlib.sha256(json.dumps(plan['sites'],sort_keys=True).encode()).hexdigest():
        raise ValueError('Static parking probe does not match current geometry or saved map')
    network['map_sha256']=maps
    network['scope']='24 parking slots (13 additional candidates pending UE entry/exit); Q3 retired'
    nodes={n['node_id']:n for n in network['nodes']};edges={e['edge_id']:e for e in network['edges']}
    added=[]
    for site in plan['sites']:
        key=site['id'];nodes[key]=dict(node_id=key,node_type='WAYPOINT',position=site['rear_xy_m'])
        row=next(r for r in probe['rows'] if r['id']==key)
        node={k:copy.deepcopy(site[k]) for k in ('kind','capacity','xy_m','rear_xy_m','heading_deg')}
        node.update(node_id=key,ground_z_m=row['ground_z_m'],occupation_resources=['berth:'+key],
            operationally_enabled=False,departure_requires_empty=[],assigned_warehouses=['W3'] if site['region']=='south_outer' else ['W4'],
            entry_edges=[key+'_entry'],exit_edges=[key+'_exit'],entry_directions_deg=[site['heading_deg']],
            entry_modes=['forward'],entry_motion_directions_deg=[site['heading_deg']],
            validation='STATIC_PASS_UE_ENTRY_EXIT_PENDING')
        network['service_nodes'][key]=node
        for mode,frm,to,phase in (('entry_rear_path',site['entry_gate'],key,'waiting_entry'),('exit_rear_path',key,site['exit_gate'],'waiting_exit')):
            path=site[mode];eid=key+('_entry' if mode=='entry_rear_path' else '_exit')
            assert math.dist(nodes[frm]['position'],path[0])<1e-6 and math.dist(nodes[to]['position'],path[-1])<1e-6
            distance=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
            e=dict(edge_id=eid,from_node=frm,to_node=to,centerline=path,width=3.8,
                speed_limit=1.5,planning_speed_limit_m_s=1.5,storage_capacity=1,lane_side='SINGLE',
                kind='service_access',maneuver='STRAIGHT',resource=None,capacity_alias=eid,
                required_resources=[],reverse=False,attachment_validation_required=False,
                access_validation_required=True,length_m=distance,estimated_travel_time_s=distance/1.5,
                must_be_empty_nodes=[],site=key,phase=phase,maximum_path_error_m=.7)
            added.append(e);edges[eid]=e
            network['access_sequences'].append(dict(site=key,source_evidence=str(PLAN.relative_to(ROOT)),edge_ids=[eid],access_validation_required=True,physical_scope='New forward-through parking entry/exit pending physical validation'))
    network['nodes']=list(nodes.values());network['edges']=list(edges.values())
    # Only join new access to its actual directed gate. Existing road movements stay intact.
    pairs={(m['incoming_edge'],m['outgoing_edge']) for m in network['movements']}
    for a in edges.values():
        for b in added if a not in added else edges.values():
            if a['to_node']!=b['from_node'] or (a['edge_id'],b['edge_id']) in pairs:continue
            network['movements'].append(dict(movement_id=a['edge_id']+'__'+b['edge_id'],incoming_edge=a['edge_id'],outgoing_edge=b['edge_id'],maneuver='STRAIGHT',reference_path=b['centerline'],conflict_zones=[],speed_limit=min(a['speed_limit'],b['speed_limit'])))
            pairs.add((a['edge_id'],b['edge_id']))
    target=ROOT/'config/topology/industrialarea_ue_parking24_network.json';target.write_text(json.dumps(network,indent=2)+'\n')
    build_scan(source=target,expected_sites=[s['id'] for s in plan['sites']],output_stem='industrialarea_ue_parking_expansion_scan',allow_candidate=True)
    sp=ROOT/'config/runtime/industrialarea_ue_parking_expansion_scan.json';spec=json.loads(sp.read_text())
    spec['parking_static_probe_sha256']=hashlib.sha256(probe_path.read_bytes()).hexdigest();spec['service_s']=1
    sp.write_text(json.dumps(spec,indent=2)+'\n')
    # Keep the speed-adaptive existing map and add access, without claiming old UE evidence matches a new map.
    transport=json.loads((ROOT/'config/topology/industrialarea_ue_fleet16_tracking_transport.json').read_text())
    old_nodes={n['node_id'] for n in transport['nodes']}
    transport['nodes'] += [copy.deepcopy(n) for n in nodes.values() if n['node_id'] not in old_nodes]
    from logistics_park.map.geometry import Polyline
    from collections import defaultdict
    parts=[]
    for e in added:
        path=Polyline(tuple(map(tuple,e['centerline'])))
        length=path.length;count=max(1,math.ceil(length/5));stations=[0.]
        for a,b in zip(path.points,path.points[1:]):stations.append(stations[-1]+math.dist(a,b))
        for i in range(count):
            lo,hi=length*i/count,length*(i+1)/count
            points=[list(path.at(lo)[0])]+[list(p) for p,t in zip(path.points,stations) if lo+1e-7<t<hi-1e-7]+[list(path.at(hi)[0])]
            frm=e['from_node'] if i==0 else e['edge_id']+'__point'+str(i)
            to=e['to_node'] if i==count-1 else e['edge_id']+'__point'+str(i+1)
            row=copy.deepcopy(e);row.update(edge_id=e['edge_id']+'__test'+str(i),from_node=frm,to_node=to,centerline=points)
            parts.append(row)
            if i<count-1:transport['nodes'].append(dict(node_id=to,node_type='WAYPOINT',position=points[-1]))
        transport['physical_centerlines'][e['edge_id']]=e['centerline'];transport['physical_capacities'][e['edge_id']]=1
    transport['edges'] += parts
    # Join real <=5m slices, including the original road boundary pieces.
    outgoing=defaultdict(list);new_outgoing=defaultdict(list)
    for e in transport['edges']:outgoing[e['from_node']].append(e)
    for e in parts:new_outgoing[e['from_node']].append(e)
    ids={e['edge_id'] for e in parts}
    existing={(m['incoming_edge'],m['outgoing_edge']) for m in transport['movements']}
    for a in transport['edges']:
        for b in (outgoing if a['edge_id'] in ids else new_outgoing)[a['to_node']]:
            pair=(a['edge_id'],b['edge_id'])
            if pair in existing:continue
            transport['movements'].append(dict(movement_id=a['edge_id']+'__'+b['edge_id'],incoming_edge=a['edge_id'],outgoing_edge=b['edge_id'],maneuver='STRAIGHT',reference_path=b['centerline'],conflict_zones=[],speed_limit=min(a['speed_limit'],b['speed_limit'])))
            existing.add(pair)
    out=ROOT/'config/topology/industrialarea_ue_parking24_transport.json';out.write_text(json.dumps(transport,indent=2)+'\n')
    plan.update(current_map_sha256=maps,static_probe_status='PASS',source_static_probe_sha256=spec['parking_static_probe_sha256'],candidate_network=str(target.relative_to(ROOT)),candidate_transport=str(out.relative_to(ROOT)))
    PLAN.write_text(json.dumps(plan,indent=2)+'\n')
    print('Compiled 24 parking nodes; new access disabled until UE tests pass')


def activate(evidence_dir):
    """Promote geometry only after every new slot has matched physical evidence."""
    from logistics_park.routing.service_network import ServiceNetwork
    plan=json.loads(PLAN.read_text())
    spec_path=ROOT/'config/runtime/industrialarea_ue_parking_expansion_scan.json'
    spec=json.loads(spec_path.read_text())
    report_path=Path(evidence_dir)/'waiting_acceptance.json'
    report=json.loads(report_path.read_text())
    if report['status']=='FAIL' and (Path(evidence_dir)/'waiting_recovered_acceptance.json').exists():
        recovered_path=Path(evidence_dir)/'waiting_recovered_acceptance.json'
        recovered=json.loads(recovered_path.read_text())
        meta=recovered['restoration_recovery'];recovery_path=ROOT/meta['report']
        recovery=json.loads(recovery_path.read_text())
        if (hashlib.sha256(recovery_path.read_bytes()).hexdigest()!=meta['sha256']
                or recovery['status']!='PASS' or not recovery['saved_maps_unchanged']
                or recovery['original_scan_sha256']!=hashlib.sha256(report_path.read_bytes()).hexdigest()
                or not recovery['observations'] or any(r['maximum_world_xy_error_m']>=.1 or r['maximum_speed_m_s']>=.1 or not r['api_restored'] for r in recovery['observations'])
                or any(recovered[k]!=report[k] for k in ('results','map_sha256','scenario_sha256','topology_sha256','completed_pairs','failed_pairs','body_obstacle_contacts','vehicle_collisions'))):
            raise ValueError('Restoration recovery is not bound to the original route evidence')
        report_path=recovered_path;report=recovered
    expected={s['id']+'__local_access' for s in plan['sites']}
    passed={r['pair_id'] for r in report['results'] if r['status']=='PASS'}
    if (report['status']!='PASS' or passed!=expected or len(report['results'])!=len(expected)
            or report.get('completed_pairs')!=len(expected) or report.get('failed_pairs')!=0
            or not report.get('ue_physics_validated') or report.get('body_obstacle_contacts')!=0
            or report.get('vehicle_collisions')!=0 or not report.get('fleet_restored')
            or not report.get('api_restored') or not report.get('saved_maps_unchanged')
            or report.get('map_sha256')!=plan['current_map_sha256']
            or report.get('topology_sha256')!=hashlib.sha256((ROOT/spec['map']).read_bytes()).hexdigest()
            or report.get('scenario_sha256')!=hashlib.sha256(spec_path.read_bytes()).hexdigest()):
        raise ValueError('Complete, matching and restored physical parking test required')
    current_maps={n:hashlib.sha256((Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')/n).read_bytes()).hexdigest() for n in plan['current_map_sha256']}
    if current_maps!=plan['current_map_sha256']:raise ValueError('Saved UE map changed after parking test')
    target=ROOT/plan['candidate_network'];network=json.loads(target.read_text())
    candidate_hash=hashlib.sha256(target.read_bytes()).hexdigest()
    if candidate_hash!=spec['service_network_sha256']:raise ValueError('Candidate service network differs from tested manifest')
    for site in plan['sites']:
        q=site['id'];node=network['service_nodes'][q]
        node.update(operationally_enabled=True,validation='UE_ENTRY_STOP_EXIT_PASS')
        site.update(operationally_enabled=True,physical_access_status='PASS')
    for q in plan['existing_waiting_nodes']:
        network['service_nodes'][q]['operationally_enabled']=True
    new_ids={s['id'] for s in plan['sites']}
    for e in network['edges']:
        if e.get('site') in new_ids:e.update(access_validation_required=False,validation='UE_ENTRY_STOP_EXIT_PASS')
    for sequence in network['access_sequences']:
        if sequence['site'] in new_ids:sequence.update(access_validation_required=False,physical_scope='Matched UE forward entry/stop/exit evidence')
    network['parking_expansion_acceptance']=dict(report=str(report_path.resolve().relative_to(ROOT)),report_sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),tested_candidate_network_sha256=candidate_hash,static_probe_sha256=spec['parking_static_probe_sha256'],map_sha256=current_maps)
    network['scope']='24 enabled waiting parking bays; Q3 retired; original access geometry preserved'
    target.write_text(json.dumps(network,indent=2)+'\n')
    transport_path=ROOT/plan['candidate_transport'];transport=json.loads(transport_path.read_text())
    for e in transport['edges']:
        if e.get('site') in new_ids:e.update(access_validation_required=False,validation='UE_ENTRY_STOP_EXIT_PASS')
    transport['parking_expansion_acceptance']=network['parking_expansion_acceptance']
    transport_path.write_text(json.dumps(transport,indent=2)+'\n')
    # New run configuration; old physical evidence and older scenarios stay intact.
    scenario=json.loads((ROOT/'config/runtime/industrialarea_ue_fleet16_debug.json').read_text())
    scenario.update(map=plan['candidate_transport'],network=plan['candidate_network'],map_sha256=current_maps,
        output='runs/fleet16_parking24_round1',park_idle_vehicles=True,
        scope='16 trucks, 32 transport tasks, 24 waiting slots, real end-of-run parking')
    scenario_path=ROOT/'config/runtime/industrialarea_ue_fleet16_parking24.json'
    scenario_path.write_text(json.dumps(scenario,indent=2)+'\n')
    active=ServiceNetwork(target,blocked_edges={'access_133','access_134','access_167'})
    waiting=[q for q,n in active.services.items() if n['kind']=='waiting']
    if len(waiting)!=24:raise ValueError('Unexpected active parking capacity')
    for q in waiting:
        if not active.route('W3-loading',q) or not active.route(q,'W4-unloading'):
            raise ValueError('Unreachable enabled parking '+q)
    plan.update(status='PASS',validated_capacity=24,physical_acceptance=network['parking_expansion_acceptance'],
        active_scenario=str(scenario_path.relative_to(ROOT)))
    PLAN.write_text(json.dumps(plan,indent=2)+'\n')
    print('Activated 24 waiting bays and real end-of-task parking:',scenario_path.relative_to(ROOT))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--compile',action='store_true')
    modes.add_argument('--activate',type=Path,metavar='EVIDENCE_DIR')
    args=parser.parse_args()
    if args.compile:compile_access()
    elif args.activate:activate(args.activate)
    else:build()
