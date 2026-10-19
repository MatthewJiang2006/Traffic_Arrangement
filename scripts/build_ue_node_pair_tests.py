#!/usr/bin/env python3
"""All 132 directed loading/unloading pairs; no waiting-node test endpoints."""
import copy,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.safety.collision import OBB,overlaps
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.runtime.transport_control import TransportController,envelope
from logistics_park.runtime.node_pair_validation import can_stage,clear_collected
from logistics_park.common.messages import VehicleState


def bounds_union(a,b):return [min(a[0],b[0]),min(a[1],b[1]),max(a[2],b[2]),max(a[3],b[3])]
def box(bounds):
    a,b,c,d=bounds;return OBB((a+c)/2,(b+d)/2,0,c-a,d-b)


def build():
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json')
    base=json.loads((ROOT/'config/topology/industrialarea_ue_transport.json').read_text())
    ids=[f'W{i}-{role}' for role in ('loading','unloading') for i in range(1,7)]
    routes_by_pair={(a,b):network.route(a,b) for a in ids for b in ids if a!=b}
    used={k for r in routes_by_pair.values() for k in r['edge_ids']}
    regions=[dict(bounds=v['bounds'],sources=[k]) for k,v in base['control_zones'].items()]
    for site in ['W1','W2','W3','W4','W5_W6_merge']:
        key='access:'+site
        points=[p for e in network.edges.values() if e['edge_id'] in used and e['kind']=='service_access' and key in e['required_resources'] for p in e['centerline']]
        if not points:continue
        bounds=[min(p[0] for p in points)-5,min(p[1] for p in points)-5,max(p[0] for p in points)+5,max(p[1] for p in points)+5]
        if site=='W5_W6_merge':bounds=bounds_union(bounds,[-45,168,-6.7,196])
        regions.append(dict(bounds=bounds,sources=[key]))
    # Merge neighbouring critical areas with insufficient room to stop a
    # complete truck between them. This avoids holding one while blocking another.
    changed=True
    while changed:
        changed=False
        for i,a in enumerate(regions):
            for j in range(i+1,len(regions)):
                b=regions[j]
                if overlaps(box(a['bounds']),box(b['bounds']),6):
                    a['bounds']=bounds_union(a['bounds'],b['bounds']);a['sources']+=b['sources'];regions.pop(j);changed=True;break
            if changed:break
    regions={f'test_region_{i}':r for i,r in enumerate(regions)}
    edges=[];nodes={};parts={};physical=copy.deepcopy(base['physical_centerlines']);capacities=copy.deepcopy(base['physical_capacities'])
    for key,e in network.edges.items():
        if e['kind']=='service_access' and e['phase'].endswith('attachment') and e['length_m']<.15:continue
        points=e['centerline'];total=e['length_m'];parts[key]=[];line=__import__('logistics_park.map.geometry',fromlist=['Polyline']).Polyline(tuple(map(tuple,points)))
        steps=max(1,math.ceil(total/5))
        for j in range(steps):
            lo,hi=total*j/steps,total*(j+1)/steps
            a=line.at(lo)[0];b=line.at(hi)[0];stations=[0.]
            for x,y in zip(points,points[1:]):stations.append(stations[-1]+math.dist(x,y))
            geometry=[list(a)]+[p for p,s in zip(points,stations) if lo+1e-7<s<hi-1e-7]+[list(b)]
            row=copy.deepcopy(e);eid=key+f'__test{j}';frm=e['from_node'] if j==0 else key+f'__cut{j}'
            to=e['to_node'] if j==steps-1 else key+f'__cut{j+1}'
            row.update(edge_id=eid,from_node=frm,to_node=to,centerline=geometry,resource=None)
            if e.get('straight_until_m'):row['straight_until_m']=max(0,e['straight_until_m']-lo)
            if e['kind']=='road':row['speed_limit']=8
            edges.append(row);parts[key].append(eid)
            nodes[frm]=dict(node_id=frm,node_type='WAYPOINT',position=list(a));nodes[to]=dict(node_id=to,node_type='WAYPOINT',position=list(b))
            alias=e['capacity_alias'];physical.setdefault(alias,points);capacities.setdefault(alias,e['storage_capacity'])
    lookup={e['edge_id']:e for e in edges};movements=[];jobs=[]
    ids=[f'W{i}-{role}' for role in ('loading','unloading') for i in range(1,7)]
    for shift in range(1,12):
        for i,origin in enumerate(ids):
            destination=ids[(i+shift)%12];route=routes_by_pair[(origin,destination)]
            if not route:raise RuntimeError('Missing nonpending pair '+origin+' '+destination)
            chosen=[k for key in route['edge_ids'] if key in parts for k in parts[key]]
            # Remove tiny topological stubs, closing the adjoining trajectory
            # by the same sub-15 cm tolerances recorded in the source access.
            expanded=[]
            for k in chosen:
                nxt=lookup[k]
                if expanded and lookup[expanded[-1]]['to_node']!=nxt['from_node']:
                    prev=lookup[expanded[-1]]
                    gap=math.dist(prev['centerline'][-1],nxt['centerline'][0])
                    if gap>.15:raise RuntimeError('Access join too large')
                    # Snap only this itinerary's copy of the previous endpoint.
                    # A millimetre bridge has an arbitrary transverse heading,
                    # which must never determine the gear-change steering.
                    snapped=copy.deepcopy(prev);snap_id=f'pair_snap_{len(edges)}'
                    snapped.update(edge_id=snap_id,to_node=nxt['from_node'])
                    snapped['centerline'][-1]=list(nxt['centerline'][0])
                    snapped['endpoint_snap_m']=gap
                    edges.append(snapped);lookup[snap_id]=snapped
                    expanded[-1]=snap_id
                expanded.append(k)
            cps=[]
            for j,(a,b) in enumerate(zip(expanded,expanded[1:]),1):
                if bool(lookup[a].get('reverse'))!=bool(lookup[b].get('reverse')):
                    cps.append(dict(event='GEAR_CHANGE',after_edge_count=j,direction=-1 if lookup[b].get('reverse') else 1))
                movements.append(dict(movement_id=a+'__'+b,incoming_edge=a,outgoing_edge=b,maneuver='STRAIGHT',reference_path=lookup[b]['centerline'],conflict_zones=[],speed_limit=lookup[b]['speed_limit']))
            cps.append(dict(event='ARRIVAL',after_edge_count=len(expanded),leg=0,site=destination,node=lookup[expanded[-1]]['to_node']))
            jobs.append(dict(pair_id=origin+'__'+destination,origin=origin,destination=destination,edges=expanded,checkpoints=cps,distance_m=route['distance_m']))
    for key,region in regions.items():
        area=box(region['bounds']);members=[]
        for e in edges:
            path=e['centerline'];x,y=path[len(path)//2];a,b=path[0],path[-1]
            h=math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if e.get('reverse') else 0)
            if overlaps(OBB(x+.75*math.cos(h),y+.75*math.sin(h),h,8.344,2.6),area,.25):members.append(e['edge_id'])
        region.update(edge_ids=members,request_ahead_m=80,approach_ahead_m=80,stop_before_m=.5,description='Test fixture FIFO critical area, released after tail clears or completed truck is physically removed')
    unique={(m['incoming_edge'],m['outgoing_edge']):m for m in movements}
    raw=dict(nodes=list(nodes.values()),edges=edges,movements=list(unique.values()),sites={},control_zones={},
        shared_resources=regions,physical_centerlines=physical,physical_capacities=capacities,
        speed_limits=dict(straight_m_s=8,turn_m_s=1.5,uturn_m_s=1,approach_deceleration_m_s2=.6))
    mp=ROOT/'config/topology/industrialarea_ue_node_pair_tests.json';mp.write_text(json.dumps(raw,indent=2)+'\n')
    output=dict(map=str(mp.relative_to(ROOT)),node_ids=ids,expected_pair_count=132,jobs=jobs,vehicles=12,
        scope='All loading/unloading node pairs; waiting nodes are not endpoints; reversible test staging only')
    (ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').write_text(json.dumps(output,indent=2)+'\n')
    print('NODE PAIRS',len(jobs),'regions',len(regions),'edges',len(edges))


if __name__=='__main__':build()
