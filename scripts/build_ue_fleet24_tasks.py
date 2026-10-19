#!/usr/bin/env python3
"""Stage 24 trucks on unchanged validated road slices, leaving service bays free."""
import copy,json,math,sys,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_control import envelope,zone_box
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps
from logistics_park.routing.service_network import ServiceNetwork

def build():
    base=json.loads((ROOT/'config/runtime/industrialarea_ue_continuous_tasks.json').read_text())
    network=json.loads((ROOT/'config/topology/industrialarea_ue_service_network.json').read_text())
    model=json.loads((ROOT/base['map']).read_text());pieces={e['edge_id']:e for e in model['edges']}
    zones=[zone_box(z) for z in model['shared_resources'].values()]
    base_network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json',blocked_edges=base['blocked_candidate_edges'])
    reachable=[base_network.reachable_edges(base_network.incoming[f'W{i}-loading'],reverse=True) for i in range(1,7)]
    bays=[envelope(VehicleState(n,*v['rear_xy_m'],math.radians(v['heading_deg']))) for n,v in network['service_nodes'].items()]
    candidates=[]
    for e in network['edges']:
        if e['kind']!='road' or len(e['centerline'])!=2 or not all(e['edge_id'] in r for r in reachable):continue
        parts=sorted([k for k in pieces if k.startswith(e['edge_id']+'__test') and k.split('__test')[-1].isdigit()],key=lambda k:int(k.split('__test')[-1]))
        for i,key in enumerate(parts[:-2]):
            p=pieces[key]['centerline'][0];q=e['centerline'][-1];h=math.atan2(q[1]-p[1],q[0]-p[0]);box=envelope(VehicleState('start',*p,h))
            if any(overlaps(box,b,.5) for b in zones+bays):continue
            candidates.append((e,parts,i,p,h,box))
    selected=[]
    while len(selected)<24:
        allowed=[c for c in candidates if all(not overlaps(c[5],s[5],1.) for s in selected)]
        if not allowed:raise RuntimeError('Insufficient separated road starts: '+str(len(selected)))
        chosen=max(allowed,key=lambda c:min((math.dist(c[3],s[3]) for s in selected),default=0))
        selected.append(chosen);candidates.remove(chosen)
    positions={}; starts=[]
    for index,(source,parts,offset,p,h,box) in enumerate(selected):
        node=f'fleet24-start-{index:02d}';vehicle=f'Car_{index//3}_{index%3}';key=f'fleet24_depart_{index:02d}'
        positions[vehicle]=node
        network['nodes'].append(dict(node_id=node,node_type='WAYPOINT',position=p))
        model['nodes'].append(dict(node_id=node,node_type='WAYPOINT',position=p))
        network['service_nodes'][node]=dict(kind='staging',capacity=1,rear_xy_m=p,heading_deg=math.degrees(h),occupation_resources=[],departure_requires_empty=[])
        e=copy.deepcopy(source);length=math.dist(p,source['centerline'][-1]);e.update(edge_id=key,from_node=node,centerline=[p,source['centerline'][-1]],length_m=length,estimated_travel_time_s=length/8.)
        network['edges'].append(e)
        for movement in list(network['movements']):
            if movement['incoming_edge']==source['edge_id']:network['movements'].append({**movement,'incoming_edge':key})
        for j,oldkey in enumerate(parts[offset:]):
            part=copy.deepcopy(pieces[oldkey]);newkey=key+'__test'+str(j);part.update(edge_id=newkey)
            if j==0:part['from_node']=node
            model['edges'].append(part)
            for z in model['shared_resources'].values():
                if oldkey in z['edge_ids']:z['edge_ids'].append(newkey)
        starts.append(dict(vehicle=vehicle,node=node,source_edge=source['edge_id'],source_slice=offset,rear_xy_m=p,heading_deg=math.degrees(h)))
    netpath='config/topology/industrialarea_ue_fleet24_service_network.json';mappath='config/topology/industrialarea_ue_fleet24_transport.json'
    (ROOT/netpath).write_text(json.dumps(network,indent=2)+'\n');(ROOT/mappath).write_text(json.dumps(model,indent=2)+'\n')
    tasks=[[f'W{i+1}-loading',f'W{(i+shift)%6+1}-unloading'] for shift in (2,3,4,1,5,2,3,4,1,5,2,3) for i in range(6)]
    spec={**base,'network':netpath,'map':mappath,'output':'runs/fleet24_tasks','positions':positions,'tasks':tasks,'light_max_sim_s':7200,'light_dt_s':.1,'physical_limit_s':600,'stall_limit_s':120,'scope':'24 retained trucks, 72 continuous tasks across six warehouses; standard movements only','staging':starts}
    (ROOT/'config/runtime/industrialarea_ue_fleet24_tasks.json').write_text(json.dumps(spec,indent=2)+'\n')
    print('FLEET24',len(positions),'vehicles',len(tasks),'tasks; unchanged road geometry')
if __name__=='__main__':build()
