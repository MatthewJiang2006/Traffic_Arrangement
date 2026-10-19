#!/usr/bin/env python3
"""Enable inside bays only after exact-route UE and occupied-neighbour checks."""
import hashlib,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.geometry import Polyline
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.safety.collision import OBB,overlaps

def fingerprints():
    folder=Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')
    return {name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in ('IndustrialArea（复件）.umap','IndustrialArea_Copy.umap')}

def route_hash(plan):
    fields=('centerline','reverse','kind','maneuver','speed_limit','straight_until_m','maximum_path_error_m')
    geometry=[{k:plan.model.edges[e].get(k) for k in fields} for e in plan.edges]
    return hashlib.sha256(json.dumps(dict(geometry=geometry,checkpoints=plan.checkpoints),sort_keys=True).encode()).hexdigest()

def main():
    netpath=ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json'
    net=json.loads(netpath.read_text());maps=fingerprints()
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_internal_waiting_connected_scan.json').read_text())
    model=TransportMap(ROOT/spec['map']);evidence={}
    reports=['runs/internal_waiting_final_ue/waiting_acceptance.json','runs/internal_waiting_Q36_connected_ue/waiting_acceptance.json']
    for filename in reports:
        report=json.loads((ROOT/filename).read_text())
        if report['status']!='PASS' or report['map_sha256']!=maps or not report.get('fleet_restored'):
            raise ValueError('Missing current successful UE evidence: '+filename)
        for row in report['results']:evidence[row['pair_id']]=(filename,row)
    for job in spec['jobs']:
        filename,row=evidence[job['pair_id']]
        plan=RouteGeometry(model,job['edges'],job['checkpoints'],job['origin'],job['stops'])
        if row['route_sha256']!=route_hash(plan) or not row['ue_physics_validated']:
            raise ValueError('UE geometry mismatch '+job['pair_id'])
    bays=spec['expected_waiting_nodes'];waiting={k:v for k,v in net['service_nodes'].items() if v['kind']=='waiting'}
    # The exact admitted ingress is reversed on exit, so the same body poses
    # cover both directions. Test every other bay occupied, with 35 cm margin.
    neighbours=[]
    for edge in net['edges']:
        site=edge.get('site')
        if site not in bays or edge['phase']!='waiting_entry':continue
        line=Polyline(tuple(map(tuple,edge['centerline'])))
        for index in range(math.ceil(line.length/.1)+1):
            (x,y),heading=line.at(min(index*.1,line.length))
            heading+=math.pi if edge.get('reverse') else 0
            body=OBB(x+.75*math.cos(heading),y+.75*math.sin(heading),heading,8.344,2.6)
            for key,peer in waiting.items():
                if key==site:continue
                px,py=peer['rear_xy_m'];h=math.radians(peer['heading_deg'])
                parked=OBB(px+.75*math.cos(h),py+.75*math.sin(h),h,8.344,2.6)
                if overlaps(body,parked,.35):raise ValueError('Occupied neighbour blocks '+site+': '+key)
        neighbours.append(dict(site=site,status='PASS'))
    for edge in net['edges']:
        if edge.get('site') in bays:edge['access_validation_required']=False
    for site in bays:
        net['service_nodes'][site]['operationally_enabled']=True
        net['service_nodes'][site]['ue_access_evidence']=evidence[site+'__local_access'][0]
    plan=json.loads((ROOT/'config/planning/ue_internal_waiting_expansion.json').read_text())
    for site in plan['sites']:
        net['service_nodes'][site['id']]['xy_m']=site['xy_m']
    retired={row['id']:row for row in net['retired_nodes'] if isinstance(row,dict)}
    retired['Q35']=dict(id='Q35',reason='No safe legal road ingress')
    for index in range(13,26):retired[f'Q{index}']=dict(id=f'Q{index}',reason='Outside park; removed from current dispatch')
    net['retired_nodes']=list(retired.values())
    net['map_sha256']=maps;net['scope']='22 inside-park waiting bays; all new local road ingress/egress UE validated'
    netpath.write_text(json.dumps(net,indent=2)+'\n')
    path=ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json';raw=json.loads(path.read_text())
    for edge in raw['edges']:
        if edge.get('site') in bays:edge['access_validation_required']=False
    path.write_text(json.dumps(raw,indent=2)+'\n')
    for count in (4,16):
        path=ROOT/f'config/runtime/industrialarea_ue_fleet{count}_flow_optimized.json';raw=json.loads(path.read_text())
        raw.update(network=str(netpath.relative_to(ROOT)),map='config/topology/industrialarea_ue_internal_waiting_transport.json',map_sha256=maps,output=f'runs/fleet{count}_inside_admission_round14')
        path.write_text(json.dumps(raw,indent=2)+'\n')
    path=ROOT/'config/planning/ue_internal_waiting_expansion.json';raw=json.loads(path.read_text());raw['status']='UE_LOCAL_ACCESS_PASS';raw['enabled_inside_capacity']=len(waiting)
    for site in raw['sites']:site.update(operationally_enabled=True,status='UE_LOCAL_ACCESS_PASS')
    path.write_text(json.dumps(raw,indent=2)+'\n')
    output=ROOT/'runs/internal_waiting_final_ue/inside_waiting_acceptance.json'
    output.write_text(json.dumps(dict(status='PASS',enabled_inside_capacity=len(waiting),new_bays=bays,retired_outer_bays=[f'Q{i}' for i in range(13,26)],map_sha256=maps,neighbours=neighbours,neighbour_margin_m=.35,evidence=reports,scope='Bay ingress/stop/egress; separate from fleet transport acceptance'),indent=2)+'\n')
    print('Enabled',len(waiting),'inside bays; matched all physical route fingerprints')
if __name__=='__main__':main()
