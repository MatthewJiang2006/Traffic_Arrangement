#!/usr/bin/env python3
"""Compile 12 local waiting ingress/egress loops; no service-node pair sweep."""
import copy,hashlib,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.map.geometry import Polyline
from logistics_park.runtime.transport_routes import RouteGeometry,TransportMap
from logistics_park.safety.collision import OBB,overlaps
from build_ue_node_pair_tests import bounds_union,box

def build(source=None, expected_sites=None, output_stem='industrialarea_ue_waiting_scan', allow_candidate=False):
    source=source or ROOT/'config/topology/industrialarea_ue_service_network.json';network=ServiceNetwork(source)
    nodes={};edges=[];jobs=[];areas=[];physical={};capacities={}
    for site in (expected_sites or [f'Q{i}' for i in range(1,13)]):
        sequences=[s for s in network.raw['access_sequences'] if s['site']==site]
        entry=next(s for s in sequences if any(network.edges[k]['phase']=='waiting_entry' for k in s['edge_ids']))
        exit=next(s for s in sequences if any(network.edges[k]['phase'] in ('waiting_exit','forward_exit','reverse_exit') for k in s['edge_ids']))
        chosen=[(network.edges[k],leg) for leg,sequence in enumerate((entry,exit)) for k in sequence['edge_ids']
                if not network.edges[k]['phase'].endswith('attachment')]
        route=[];cps=[];rows=[]
        for index,(e,leg) in enumerate(chosen):
            if e.get('access_validation_required') and not allow_candidate:raise RuntimeError('Pending local access '+site)
            if rows:
                gap=math.dist(rows[-1]['centerline'][-1],e['centerline'][0])
                if gap>1e-6:raise RuntimeError('Discontinuous local maneuver '+site)
                if leg!=chosen[index-1][1]:
                    cps.append(dict(event='ARRIVAL',after_edge_count=len(route),leg=0,site=site,node=site))
                if bool(e.get('reverse'))!=bool(chosen[index-1][0].get('reverse')):
                    cps.append(dict(event='GEAR_CHANGE',after_edge_count=len(route),direction=-1 if e.get('reverse') else 1))
            line=Polyline(tuple(map(tuple,e['centerline'])));length=line.length;count=max(1,math.ceil(length/5))
            stations=[0.]
            for a,b in zip(line.points,line.points[1:]):stations.append(stations[-1]+math.dist(a,b))
            for part in range(count):
                lo,hi=length*part/count,length*(part+1)/count
                points=[list(line.at(lo)[0])]+[list(p) for p,s in zip(line.points,stations) if lo+1e-7<s<hi-1e-7]+[list(line.at(hi)[0])]
                row=copy.deepcopy(e);key=f'{site}__{index}_{part}'
                frm=f'{site}__point{len(route)}';to=f'{site}__point{len(route)+1}'
                row.update(edge_id=key,from_node=frm,to_node=to,centerline=points,resource=None,speed_limit=min(2,e['speed_limit']),storage_capacity=1)
                row['capacity_alias']=key
                if e.get('straight_until_m'):row['straight_until_m']=max(0,e['straight_until_m']-lo)
                row['maximum_path_error_m']=e.get('maximum_path_error_m',.7)
                edges.append(row);rows.append(row);route.append(key)
                nodes[frm]=dict(node_id=frm,node_type='WAYPOINT',position=points[0]);nodes[to]=dict(node_id=to,node_type='WAYPOINT',position=points[-1])
                physical[key]=points;capacities[key]=1
        cps.append(dict(event='ARRIVAL',after_edge_count=len(route),leg=1,site=site+'-road',node=rows[-1]['to_node']))
        points=[p for row in rows for p in row['centerline']]
        bounds=[min(p[0] for p in points)-5.5,min(p[1] for p in points)-5.5,max(p[0] for p in points)+5.5,max(p[1] for p in points)+5.5]
        areas.append(dict(bounds=bounds,sites=[site]))
        jobs.append(dict(pair_id=site+'__local_access',origin=site+'-road-start',destination=site+'-road',waiting_node=site,
            stops=[site,site+'-road'],edges=route,checkpoints=cps,distance_m=sum(math.dist(a,b) for r in rows for a,b in zip(r['centerline'],r['centerline'][1:]))))
    changed=True
    while changed:
        changed=False
        for i,a in enumerate(areas):
            for j in range(i+1,len(areas)):
                b=areas[j]
                if overlaps(box(a['bounds']),box(b['bounds']),0):
                    a['bounds']=bounds_union(a['bounds'],b['bounds']);a['sites']+=b['sites'];areas.pop(j);changed=True;break
            if changed:break
    shared={f'waiting_region_{i}':dict(**a,edge_ids=[e['edge_id'] for e in edges if e['site'] in a['sites']],request_ahead_m=200,
        approach_ahead_m=200,stop_before_m=.5) for i,a in enumerate(areas)}
    lookup={e['edge_id']:e for e in edges}
    movements=[dict(movement_id=a+'__'+b,incoming_edge=a,outgoing_edge=b,maneuver='STRAIGHT',reference_path=lookup[b]['centerline'],conflict_zones=[],speed_limit=2)
               for j in jobs for a,b in zip(j['edges'],j['edges'][1:])]
    raw=dict(nodes=list(nodes.values()),edges=edges,movements=movements,sites={},control_zones={},shared_resources=shared,
        physical_centerlines=physical,physical_capacities=capacities,speed_limits=dict(straight_m_s=2,turn_m_s=1.5,uturn_m_s=1,approach_deceleration_m_s2=.6))
    map_path=ROOT/f'config/topology/{output_stem}.json';map_path.write_text(json.dumps(raw,indent=2)+'\n')
    # The scope starts/ends on the local tested approach, not graph-port snapping.
    spec=dict(map=str(map_path.relative_to(ROOT)),jobs=jobs,service_s=3,expected_waiting_nodes=expected_sites or [f'Q{i}' for i in range(1,13)],service_network=str(source.relative_to(ROOT)),
        service_network_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        map_sha256={name:hashlib.sha256((Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')/name).read_bytes()).hexdigest()
                    for name in ('IndustrialArea（复件）.umap','IndustrialArea_Copy.umap')},
        original_access_map_sha256=network.raw['map_sha256'],
        scope='Local waiting ingress/stop/egress loops; no all-pairs sweep')
    (ROOT/f'config/runtime/{output_stem}.json').write_text(json.dumps(spec,indent=2)+'\n')
    model=TransportMap(map_path)
    for job in jobs:
        p=RouteGeometry(model,job['edges'],job['checkpoints'],job['origin'],job['stops'])
        model.graph.validate_route(p.edges)
    print(len(jobs),'waiting loops',len(edges),'segments',len(areas),'independent regions',round(sum(j['distance_m'] for j in jobs),1),'m')

if __name__=='__main__':build()
