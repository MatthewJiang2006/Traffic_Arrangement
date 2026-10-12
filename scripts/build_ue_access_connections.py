#!/usr/bin/env python3
"""Smooth access boundaries and group spatially separated physical trials."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from logistics_park.map.geometry import Polyline
from logistics_park.runtime.transport_control import TransportController, envelope
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.ue_transport import configured_plans
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps

D = 2.3285 / 2
OUT = ROOT / 'runs/ue_access_connections'


def rear(path):
    return [[x-D*math.cos(math.radians(h)), y-D*math.sin(math.radians(h))] for x,y,h in path]


def blend(a, b, ha, hb):
    """Quintic Bezier with matched endpoint tangent and zero endpoint curvature."""
    distance=math.dist(a,b)
    u=[math.cos(math.radians(ha)),math.sin(math.radians(ha))]
    v=[math.cos(math.radians(hb)),math.sin(math.radians(hb))]
    controls=[a, [a[i]+distance*u[i]/5 for i in (0,1)],
              [a[i]+2*distance*u[i]/5 for i in (0,1)],
              [b[i]-2*distance*v[i]/5 for i in (0,1)],
              [b[i]-distance*v[i]/5 for i in (0,1)], b]
    return [[sum(math.comb(5,j)*(1-t)**(5-j)*t**j*controls[j][i] for j in range(6))
             for i in (0,1)] for t in [k/80 for k in range(81)]]


def curvature(points):
    values=[]
    for a,b,c in zip(points,points[1:],points[2:]):
        denom=math.dist(a,b)*math.dist(b,c)*math.dist(c,a)
        if denom>1e-10:
            values.append(2*abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/denom)
    return max(values,default=0.)


def simulate(scenario):
    model=TransportMap(ROOT/scenario['map']);plans=configured_plans(model,scenario)
    controller=TransportController(model,plans,service_s=2)
    states={n:VehicleState(n,*p.point(0),p.heading(0)) for n,p in plans.items()}
    for n in plans:controller.activate(n,0)
    dt=.02;failure=[]
    for tick in range(25000):
        now=tick*dt
        try:commands=controller.tick(states,now)
        except RuntimeError as exc:failure.append(str(exc));break
        if all(ex.finished for ex in controller.executions.values()):break
        for n,state in states.items():
            target,delta=commands.get(n,(0.,0.));v=state.speed+max(-.6*dt,min(.6*dt,target-state.speed))
            h=state.heading+.5*v*math.tan(delta)/2.3285*dt
            state.x+=v*math.cos(h)*dt;state.y+=v*math.sin(h)*dt
            state.heading+=v*math.tan(delta)/2.3285*dt;state.speed=v;state.timestamp=now
    else:failure.append('simulation timeout')
    result=controller.report();result.update(failure=failure,ue_physics_validated=False)
    if failure:result['status']='FAIL'
    return result


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    source=ROOT/'config/topology/industrialarea_ue_service_network.json'
    graph=json.loads(source.read_text());edges={e['edge_id']:e for e in graph['edges']}
    ports={p['port_id']:p for p in graph['road_ports']}
    supplements=json.loads((ROOT/'config/planning/ue_supplementary_access_evidence.json').read_text())['sites']
    loading=json.loads((ROOT/'config/planning/ue_loading_access_routes.json').read_text())['sites']
    frozen_path=ROOT/'config/planning/ue_access_connection_candidates.json'
    frozen=json.loads(frozen_path.read_text()) if (ROOT/'config/planning/ue_access_connection_acceptance.json').exists() and frozen_path.exists() else None
    if frozen:
        candidates=frozen['connections'];cases=frozen['cases']
    else:
        candidates={}
        # Relocate each road boundary along its legal direction, so its lateral
        # offset is traversed over distance rather than a perpendicular stub.
        for pid in ['port_2','port_6','port_60','port_61','port_62','port_63','port_64','port_65']:
            p=ports[pid];entry=p['direction']=='entry';yaw=270
            reach=10 if entry else 4 if p['site'].startswith('Q') else 8
            r=p['road_reference_rear_xy_m'];a=p['access_reference_rear_xy_m']
            road=[r[0],r[1]+reach if entry else r[1]-reach]
            path=blend(road,a,yaw,p['body_heading_deg']) if entry else blend(a,road,p['body_heading_deg'],yaw)
            candidates[pid]=dict(**p,road_rear_xy_m=road,centerline=path,reverse=False,
                                maximum_curvature_m_inv=curvature(path),status='CANDIDATE')
        # Q3 reverse exit retraces the accepted entry plus the smooth lane entry.
        p=ports['port_78'];candidates['port_78']=dict(**p,
            road_rear_xy_m=candidates['port_60']['road_rear_xy_m'],
            centerline=list(reversed(candidates['port_60']['centerline'])),reverse=True,
            maximum_curvature_m_inv=candidates['port_60']['maximum_curvature_m_inv'],status='CANDIDATE')
        cases={}
        def phase(label,path,reverse=False):return dict(label=label,centerline=path,reverse=reverse)
        for wid,pid in [('W1','port_2'),('W2','port_6')]:
            cases[wid+'-exit']=[phase('existing_exit',rear(loading[wid]['exit'])),phase(pid,candidates[pid]['centerline'])]
        for q,pi,po in [('Q4','port_62','port_63'),('Q5','port_64','port_65')]:
            cases[q+'-access']=[phase(pi,candidates[pi]['centerline']),
                phase('accepted_entry',rear(supplements[q+'-waiting_entry']['path_xy_heading'])),
                phase('accepted_exit',rear(supplements[q+'-waiting_exit']['path_xy_heading'])),
                phase(po,candidates[po]['centerline'])]
        cases['Q3-forward-exit']=[phase('accepted_exit',rear(supplements['Q3-waiting_exit']['path_xy_heading'])),
                                  phase('port_61',candidates['port_61']['centerline'])]
        cases['Q3-entry-reverse-exit']=[phase('port_60',candidates['port_60']['centerline']),
            phase('accepted_entry',rear(supplements['Q3-waiting_entry']['path_xy_heading'])),
            phase('reverse_exit',list(reversed(rear(supplements['Q3-waiting_entry']['path_xy_heading']))),True),
            phase('port_78',candidates['port_78']['centerline'],True)]
        for site,access in [('W3-reverse-entry','access_164'),('W4-reverse-exit','access_165')]:
            e=edges[access];cases[site]=[phase(access,e['centerline'],True)]
        q9=[edges[k] for k in next(s for s in graph['access_sequences'] if s['site']=='Q9' and s['access_validation_required'])['edge_ids']]
        q9_path=copy.deepcopy(q9[-1]['centerline']);q9_path[0]=q9[0]['centerline'][0]
        cases['Q9-reverse-entry']=[phase('access_167',q9_path,True)]
    groups=[['W1-exit','W2-exit','Q5-access','W3-reverse-entry','Q9-reverse-entry'],
            ['Q4-access','W4-reverse-exit','Q3-forward-exit'],['Q3-entry-reverse-exit']]
    template=json.loads((ROOT/'config/runtime/industrialarea_ue_interwarehouse_1v_1cycles.json').read_text())
    summaries=[]
    for number,group in enumerate(groups,1):
        nodes={};rows=[];movements=[];routes={};physical={};capacities={}
        for index,case in enumerate(group):
            name=['Car_0_0','Car_0_1','Car_0_2','Car_1_0','Car_1_1'][index];ids=[];cps=[]
            for j,p in enumerate(cases[case]):
                key=name+'_'+str(j);frm=name+'_'+str(j)+'_node';to=name+'_'+str(j+1)+'_node'
                if j and math.dist(rows[-1]['centerline'][-1],p['centerline'][0])>1e-6:raise ValueError('disconnected '+case)
                row=dict(edge_id=key,from_node=frm,to_node=to,centerline=p['centerline'],kind='road',
                         maneuver='STRAIGHT',width=3.8,speed_limit=1 if p['reverse'] else 1.5,
                         storage_capacity=1,capacity_alias=key,reverse=p['reverse'],access_label=p['label'])
                rows.append(row);nodes[frm]=dict(node_id=frm,node_type='WAYPOINT',position=p['centerline'][0])
                nodes[to]=dict(node_id=to,node_type='WAYPOINT',position=p['centerline'][-1])
                physical[key]=p['centerline'];capacities[key]=1
                if ids:movements.append(dict(movement_id=ids[-1]+'__'+key,incoming_edge=ids[-1],outgoing_edge=key,
                    maneuver='STRAIGHT',reference_path=p['centerline'],conflict_zones=[],speed_limit=1.5))
                if j and cases[case][j-1]['reverse']!=p['reverse']:
                    cps.append(dict(event='GEAR_CHANGE',after_edge_count=j,direction=-1 if p['reverse'] else 1))
                ids.append(key)
            cps.append(dict(event='ARRIVAL',after_edge_count=len(ids),leg=0,site=case,node=to))
            routes[name]=dict(edges=ids,origin=case+'-start',stops=[case],checkpoints=cps,case=case)
        model=dict(nodes=list(nodes.values()),edges=rows,movements=movements,sites={},control_zones={},
            shared_resources={},physical_centerlines=physical,physical_capacities=capacities,
            speed_limits=dict(straight_m_s=1.5,turn_m_s=1.5,uturn_m_s=1,approach_deceleration_m_s2=.6))
        mp=ROOT/f'config/topology/industrialarea_ue_access_batch_{number}.json';mp.write_text(json.dumps(model,indent=2)+'\n')
        scenario=dict(application='shared_transport',map=str(mp.relative_to(ROOT)),routes=routes,
            scope='Spatially disjoint access acceptance; no full-park dispatch',max_runtime_s=240,
            body_prefix=f'accessbatch{number}',minimum_wait_s=0,require_parallel_motion=len(group)>1,service_s=2,
            adapter=copy.deepcopy(template['adapter']),verified_ground_actors=template['verified_ground_actors'])
        scenario['adapter']['vehicle_ids']=list(routes)
        scene_path=ROOT/'runs/ue_scene/scene.json'
        if scene_path.exists():
            scene=json.loads(scene_path.read_text())
            scenario['verified_ground_actors']=sorted({a['name'] for a in scene['actors'] if a['name'].startswith('Traffic') and any(k in a['name'] for k in ('Ramp','ToCurb','ToHardstand','LaneTaper','SeamBridge'))})
        sp=ROOT/f'config/runtime/industrialarea_ue_access_batch_{number}.json';sp.write_text(json.dumps(scenario,indent=2)+'\n')
        result=simulate(scenario);(OUT/f'lightweight_batch_{number}.json').write_text(json.dumps(result,indent=2)+'\n')
        summaries.append(dict(batch=number,cases=group,scenario=str(sp.relative_to(ROOT)),lightweight=result))
        print('BATCH',number,result['status'],result.get('failure'),flush=True)
    output=dict(schema_version=1,source_graph_sha256=frozen['source_graph_sha256'] if frozen else hashlib.sha256(source.read_bytes()).hexdigest(),
        source_map_sha256=frozen['source_map_sha256'] if frozen else graph['map_sha256'],connections=candidates,cases=cases,batches=summaries,
        scope='Candidates and lightweight evidence; promotion requires matching UE physical acceptance')
    (ROOT/'config/planning/ue_access_connection_candidates.json').write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':build()
