#!/usr/bin/env python3
"""Compile all approved bays, directed road alternatives and queue choices."""
import copy
import hashlib
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.geometry import Polyline
from logistics_park.map.map_loader import load_map
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.safety.collision import OBB,overlaps

NETWORK=ROOT/'config/topology/industrialarea_ue_service_network.json'
CATALOG=ROOT/'config/planning/ue_service_connectivity.json'
D=2.3285/2


def rear(p):
    a=math.radians(p[2]);return [p[0]-D*math.cos(a),p[1]-D*math.sin(a)]


def length(points):return sum(math.dist(a,b) for a,b in zip(points,points[1:]))


def sample(points,step=1.):
    for a,b in zip(points,points[1:]):
        count=max(1,math.ceil(math.dist(a,b)/step))
        for j in range(count):
            t=j/count;yield [a[i]+t*(b[i]-a[i]) for i in range(3)]
    yield points[-1]


def build():
    inputs={}
    def read(relative):
        path=ROOT/relative;inputs[relative]=hashlib.sha256(path.read_bytes()).hexdigest()
        return json.loads(path.read_text())
    base=read('config/topology/industrialarea_ue_transport.json')
    proposal=read('config/planning/ue_station_proposal.json')
    loading=read('config/planning/ue_loading_access_routes.json')
    remaining=read('config/planning/ue_remaining_service_routes.json')
    waiting=read('config/planning/ue_waiting_access_routes.json')
    validation=read('config/planning/ue_station_validation.json')
    supplemental=read('config/planning/ue_supplementary_access_evidence.json')['sites']
    approval_file='config/planning/ue_access_connection_acceptance.json'
    approval=read(approval_file) if (ROOT/approval_file).exists() else {}
    if approval:
        if approval['status']!='PASS' or approval['static_actor_preservation']['status']!='PASS':
            raise ValueError('Access approval or map preservation incomplete')
        if waiting['map_sha256'] not in (approval['map_sha256'],approval['preserved_source_map_sha256']):
            raise ValueError('Access approval belongs to a different source map')
    services={}
    for warehouse in proposal['stations']:
        for role in ('loading','unloading'):
            point=warehouse[role];key=warehouse['id']+'-'+role
            services[key]=dict(node_id=key,kind=role,warehouse=warehouse['id'],capacity=1,
                xy_m=point['xy_m'],heading_deg=point['heading_deg'],ground_z_m=point['ground_z_m'],
                rear_xy_m=rear([*point['xy_m'],point['heading_deg']]),operationally_enabled=False)
    for point in proposal['shared_waiting']:
        key=point['id'];services[key]=dict(node_id=key,kind='waiting',capacity=1,
            xy_m=point['xy_m'],heading_deg=point['heading_deg'],ground_z_m=point['ground_z_m'],
            rear_xy_m=rear([*point['xy_m'],point['heading_deg']]),
            assigned_warehouses=[w for w,qs in proposal['waiting_assignment'].items() if key in qs],
            departure_requires_empty=['Q2'] if key=='Q1' else [],operationally_enabled=False)
    nodes={n['node_id']:copy.deepcopy(n) for n in base['nodes']}
    for key,node in services.items():nodes[key]=dict(node_id=key,node_type='WAYPOINT',position=node['rear_xy_m'])
    coordinate_nodes={tuple(round(x,5) for x in n['position']):key for key,n in nodes.items()}
    base_edges={e['edge_id']:e for e in base['edges']}
    ports=[];cuts=defaultdict(list);access=[];joins=[];sequence_records=[]
    serial=0
    def ordinary_node(xy):
        key=tuple(round(x,5) for x in xy)
        if key not in coordinate_nodes:
            name=f'access_point_{len(coordinate_nodes)}';coordinate_nodes[key]=name
            nodes[name]=dict(node_id=name,node_type='WAYPOINT',position=xy)
        return coordinate_nodes[key]
    def road_port(p,direction):
        original_xy=rear(p);xy=original_xy;candidates=[]
        port_id=f'port_{len(ports)}'
        verified=approval.get('connections',{}).get(port_id)
        if verified:
            if (verified['status']!='PASS' or verified['direction']!=direction
                    or math.dist(original_xy,verified['access_reference_rear_xy_m'])>1e-6):
                raise ValueError('Approved access boundary changed '+port_id)
            xy=verified['road_rear_xy_m']
        for key,e in base_edges.items():
            if e['kind']!='road' and not (e['kind']=='junction' and e.get('maneuver')=='STRAIGHT'):continue
            path=Polyline(tuple(map(tuple,e['centerline'])));projection=path.project(*xy)
            heading=math.degrees(projection.heading)
            difference=abs((heading-p[2]+180)%360-180)
            if difference<12:candidates.append((projection.distance,key,projection.s,projection.point))
        if not candidates:raise ValueError('no same-direction road for '+str(p))
        gap,key,s,point=min(candidates)
        if gap>2.6:raise ValueError('road attachment too remote '+str(p))
        e=base_edges[key]
        if s<1e-5:node=e['from_node']
        elif s>Polyline(tuple(map(tuple,e['centerline']))).length-1e-5:node=e['to_node']
        else:
            node=ordinary_node(list(point));cuts[key].append((s,node))
        ports.append(dict(port_id=port_id,direction=direction,node_id=node,
            source_road_edge=key,station_m=s,body_heading_deg=p[2]%360,
            access_reference_rear_xy_m=original_xy,road_reference_rear_xy_m=list(point),attachment_gap_m=gap,
            attachment_validation_required=gap>.15))
        if verified:ports[-1].update(approved_connection=verified,ue_boundary_validated=True)
        return node,list(point),gap
    def resources(site):
        if site.startswith(('W5','W6')) or site in ('Q1','Q2'):return ['access:W5_W6_merge']
        if site in ('Q6','Q7','Q8'):return ['access:Q6_Q7_Q8']
        if site in ('Q9','Q10'):return ['access:Q9_Q10']
        if site in ('Q11','Q12'):return ['access:Q11_Q12']
        return ['access:'+site.split('-')[0]]
    def edge(frm,to,points,phase,reverse,source,site,gap=0.):
        nonlocal serial
        verified=ports[-1].get('approved_connection') if phase.endswith('attachment') else None
        if verified:
            points=copy.deepcopy(verified['centerline'])
            if math.dist(points[0],nodes[frm]['position'])>1e-3 or math.dist(points[-1],nodes[to]['position'])>1e-3:
                raise ValueError('Approved connection endpoint changed '+str((ports[-1]['port_id'],points[0],nodes[frm]['position'],points[-1],nodes[to]['position'])))
            points[0]=nodes[frm]['position'];points[-1]=nodes[to]['position']
            reverse=verified['reverse']
        if length(points)<1e-7:return
        speed=1 if reverse else 1.5
        key=f'access_{serial:03d}';serial+=1
        row=dict(edge_id=key,from_node=frm,to_node=to,centerline=points,width=3.3,
            speed_limit=speed,storage_capacity=1,kind='service_access',maneuver='STRAIGHT',
            reverse=reverse,phase=phase,source_evidence=source,site=site,
            required_resources=resources(site),attachment_gap_m=gap,
            attachment_validation_required=gap>.15,
            validation='ROAD_PORT_ATTACHMENT_PENDING' if gap>.15 else 'LOCAL_LOW_SPEED_ACCESS_EVIDENCE',
            capacity_alias=key)
        if phase.endswith('attachment'):
            if verified:
                row.update(validation='UE_ACCESS_CONNECTION_PASS',attachment_validation_required=False,
                           physical_evidence=verified['evidence'],geometry_usage='Verified smooth rear-axle trajectory')
            else:
                row['body_heading_deg']=ports[-1]['body_heading_deg']
                row['geometry_usage']='Topological connection only; generate a smooth driving transition before execution'
        access.append(row);return key
    def sequence(site,phases,source,pending=False):
        # Reconcile small nominal heading differences at continuous phase joins.
        first_port=len(ports)
        verified_reverse=approval.get('verified_reverse_sites',{}).get(site) if pending else None
        if verified_reverse and verified_reverse['status']!='PASS':raise ValueError('Reverse access not approved')
        pending=pending and not verified_reverse
        previous=None;ids=[]
        for index,(phase,body,reverse) in enumerate(phases):
            body=copy.deepcopy(body);points=[rear(p) for p in body]
            if previous is not None:
                gap=math.dist(points[0],nodes[previous]['position'])
                if gap>.25:raise ValueError('disconnected access phases '+site+' '+phase)
                if gap>1e-6:joins.append(dict(site=site,phase=phase,nominal_rear_gap_m=gap,tracking_join=True))
                points[0]=nodes[previous]['position'];start=previous
            else:
                start=ordinary_node(points[0])
                if start not in services:
                    anchor,xy,gap=road_port(body[0],'entry')
                    if verified_reverse and gap<.005:
                        points[0]=xy;start=anchor  # Match the tested millimetre-level lane join.
                    else:
                        key=edge(anchor,start,[xy,points[0]],'entry_attachment',False,source,site,gap)
                        if key:ids.append(key)
            if previous is not None and start not in services and phase in ('reverse_entry','waiting_entry','reverse'):
                anchor,xy,gap=road_port(body[0],'entry')
                key=edge(anchor,start,[xy,points[0]],'entry_attachment',False,source,site,gap)
                if key:ids.append(key)
            end=ordinary_node(points[-1]);markers=[(0.,start),(length(points),end)]
            path=Polyline(tuple(map(tuple,points)))
            for name,node in services.items():
                projection=path.project(*node['rear_xy_m'])
                # Interior service nodes retain their actual fixed vehicle pose.
                if projection.distance<.02 and .001<projection.s<path.length-.001:
                    nearest=min(body,key=lambda p:math.dist(rear(p),projection.point))
                    if abs((nearest[2]-node['heading_deg']+180)%360-180)<.2:
                        markers.append((projection.s,name))
            markers=sorted(set(markers));stations=[0.]
            for a,b in zip(points,points[1:]):stations.append(stations[-1]+math.dist(a,b))
            for (lo,frm),(hi,to) in zip(markers,markers[1:]):
                segment=[nodes[frm]['position']]+[p for p,s in zip(points,stations) if lo+1e-7<s<hi-1e-7]+[nodes[to]['position']]
                key=edge(frm,to,segment,phase,reverse,source,site)
                if key:ids.append(key)
            previous=end
            if end not in services and index<len(phases)-1 and 'exit' in phase:
                anchor,xy,gap=road_port(body[-1],'exit')
                key=edge(end,anchor,[points[-1],xy],'exit_attachment',False,source,site,gap)
                if key:ids.append(key)
            if index==len(phases)-1 and end not in services:
                anchor,xy,gap=road_port(body[-1],'exit')
                key=edge(end,anchor,[points[-1],xy],'exit_attachment',False,source,site,gap)
                if key:ids.append(key)
        if pending:
            for row in access:
                if row['edge_id'] in ids:
                    row['access_validation_required']=True
                    row['validation']='NEW_REVERSE_ACCESS_CANDIDATE'
        if verified_reverse:
            for row in access:
                if row['edge_id'] in ids:
                    row.update(access_validation_required=False,validation='UE_ACCESS_CONNECTION_PASS',
                               physical_evidence=verified_reverse['evidence'])
        sequence_records.append(dict(site=site,source_evidence=source,edge_ids=ids,access_validation_required=pending,
            physical_scope='existing single-truck low-speed phases; road boundary joins and compositions require separate validation'))
        for port in ports[first_port:]:
            port.update(site=site,source_evidence=source,access_validation_required=pending)
    def phases(row,names):return [(name,row[name]['path_xy_heading'],row[name].get('reverse',False)) for name in names]
    for wid,row in loading['sites'].items():
        sequence(wid+'-loading',[(n,row[n],n=='reverse') for n in ('approach','reverse')],'config/planning/ue_loading_access_routes.json')
        sequence(wid+'-loading',[(n,row[n],False) for n in ('exit','recovery')],'config/planning/ue_loading_access_routes.json')
    for site,row in remaining['sites'].items():
        entry=['entry'] if 'entry' in row else ['approach','reverse_entry']
        departure=['reverse_exit','lane_recovery'] if 'reverse_exit' in row else ['forward_exit','lane_recovery']
        sequence(site,phases(row,entry),'config/planning/ue_remaining_service_routes.json')
        sequence(site,phases(row,departure),'config/planning/ue_remaining_service_routes.json')
    for site,row in waiting['sites'].items():
        entry=(['approach'] if 'approach' in row else [])+['waiting_entry']
        departure=(['forward_to_loading','forward_to_unloading','forward_exit','lane_recovery'] if site in ('Q1','Q2')
                   else ['waiting_exit' if 'waiting_exit' in row else 'reverse_exit','lane_recovery'])
        sequence(site,phases(row,entry),'config/planning/ue_waiting_access_routes.json')
        sequence(site,phases(row,departure),'config/planning/ue_waiting_access_routes.json')
    # Q3-Q5 accepted trials record their precise paths through route_file.
    for q,suffix in [('Q3',''),('Q4','-gentle'),('Q5','')]:
        for tail,phase in [(suffix,'waiting_entry'),('-exit-gentle','waiting_exit')]:
            trial=supplemental[q+'-'+phase];source=trial['source_trial']
            if trial['status']!='PASS':raise ValueError('unaccepted waiting route '+source)
            geometry=trial['path_xy_heading']
            sequence(q,[(phase,geometry,False)],source)
    # Wide-road bays are separate stops on the measured northbound drive.
    for wid,start,finish in [('W3',54,88),('W4',260,298)]:
        trial=supplemental[wid]
        if trial['status']!='PASS':raise ValueError('roadside bay not validated')
        bays=[services[wid+'-'+r] for r in ('loading','unloading')]
        sequence(wid+'-loading',[('roadside_service',[[-97.89,start,90],*[ [*b['xy_m'],90] for b in bays],[-98,finish,90]],False)],f'runs/ue_scene/station_siting/roadside_dynamic_{wid}.json')
    # Trim measured straight portions to accessible road stations. Do not
    # force traffic to reproduce the external spawn used by a local trial.
    sequence('W3-unloading',[('measured_entry_suffix',[[-98,74,90],[-98,80,90]],False)],
             'runs/ue_scene/station_siting/roadside_dynamic_W3.json')
    sequence('W3-loading',[('measured_exit_prefix',[[-98,66,90],[-98,74,90]],False)],
             'runs/ue_scene/station_siting/roadside_dynamic_W3.json')
    sequence('W4-loading',[('measured_exit_prefix',[[-98,274,90],[-98,280,90]],False)],
             'runs/ue_scene/station_siting/roadside_dynamic_W4.json')
    sequence('W4-unloading',[('measured_entry_suffix',[[-98,280,90],[-98,288,90]],False)],
             'runs/ue_scene/station_siting/roadside_dynamic_W4.json')
    q10=waiting['sites']['Q10']['waiting_entry']
    sequence('Q10',[('measured_entry_suffix',q10['path_xy_heading'][1:],False)],
             'config/planning/ue_waiting_access_routes.json')
    # These reverse manoeuvres are planning candidates, explicitly pending
    # physical validation. They do not authorize reverse cruising on roads.
    sequence('W3-loading',[('reverse_entry_candidate',[[-97.8936,76,90],[-98,66,90]],True)],
             'derived:W3 endpoint lies below A turnaround; reserve northbound end segment',pending=True)
    sequence('W4-unloading',[('reverse_exit_candidate',[[-98,288,90],[-97.8937,278,90]],True)],
             'derived:W4 endpoint lies beyond C turnaround; reverse to existing turnaround approach',pending=True)
    q9=waiting['sites']['Q9']['waiting_exit']
    sequence('Q9',[('reverse_entry_candidate',list(reversed(q9['path_xy_heading'])),True)],
             'derived:reverse of config/planning/ue_waiting_access_routes.json Q9 waiting_exit',pending=True)
    geometry=supplemental['Q3-waiting_entry']['path_xy_heading']
    sequence('Q3',[('reverse_exit_candidate',list(reversed(geometry)),True)],
             'derived:reverse of accepted Q3 entry; merge above existing A turnaround',pending=True)
    # Split only at attachment stations, keeping every directed base road.
    edges=[];replacement={}
    for key,e in base_edges.items():
        path=Polyline(tuple(map(tuple,e['centerline'])))
        boundaries=sorted(set([(0.,e['from_node']),(path.length,e['to_node']),*cuts[key]]))
        replacement[key]=[];stations=[0.]
        for a,b in zip(e['centerline'],e['centerline'][1:]):stations.append(stations[-1]+math.dist(a,b))
        for part,((lo,frm),(hi,to)) in enumerate(zip(boundaries,boundaries[1:])):
            if hi-lo<1e-7:continue
            row=copy.deepcopy(e);row.update(edge_id=key if len(boundaries)==2 else key+f'__service_{part}',from_node=frm,to_node=to,
                centerline=[nodes[frm]['position']]+[p for p,s in zip(e['centerline'],stations) if lo+1e-7<s<hi-1e-7]+[nodes[to]['position']],
                source_road_edge=key,required_resources=[e['resource']] if e.get('resource') else [],reverse=False,
                attachment_validation_required=False,validation='EXISTING_DIRECTED_ROAD_TOPOLOGY')
            replacement[key].append(row['edge_id']);edges.append(row)
    edges+=access;lookup={e['edge_id']:e for e in edges};movements=[];legal=set()
    def movement(a,b,maneuver='STRAIGHT'):
        if (a,b) in legal:return
        legal.add((a,b));e=lookup[b]
        movements.append(dict(movement_id=a+'__'+b,incoming_edge=a,outgoing_edge=b,maneuver=maneuver,
            reference_path=e['centerline'],conflict_zones=e['required_resources'],speed_limit=e['speed_limit']))
    for row in base['movements']:movement(replacement[row['incoming_edge']][-1],replacement[row['outgoing_edge']][0],row['maneuver'])
    for parts in replacement.values():
        for a,b in zip(parts,parts[1:]):movement(a,b)
    incoming=defaultdict(list);outgoing=defaultdict(list)
    for e in edges:incoming[e['to_node']].append(e['edge_id']);outgoing[e['from_node']].append(e['edge_id'])
    for node in nodes:
        for a in incoming[node]:
            for b in outgoing[node]:
                if lookup[a]['kind']=='service_access' or lookup[b]['kind']=='service_access':movement(a,b)
    parking={key:OBB(*n['xy_m'],math.radians(n['heading_deg']),9,3.3) for key,n in services.items()}
    zones={key:OBB((v['bounds'][0]+v['bounds'][2])/2,(v['bounds'][1]+v['bounds'][3])/2,0,
                   v['bounds'][2]-v['bounds'][0],v['bounds'][3]-v['bounds'][1]) for key,v in base['control_zones'].items()}
    access_resource_specs={'access:W5_W6_merge':dict(bounds=[-45,168,-6.7,196],capacity=1,
        release_rule='all body boxes and tail clear',source='config/topology/industrialarea_ue_interwarehouse_2v_1cycles.json')}
    zones['access:W5_W6_merge']=OBB((-45-6.7)/2,(168+196)/2,0,45-6.7,196-168)
    for e in edges:
        e['length_m']=length(e['centerline'])
        speed=8 if e['kind']=='road' else e['speed_limit']
        e['planning_speed_limit_m_s']=speed;e['estimated_travel_time_s']=e['length_m']/speed
        empty=set();required=set(e['required_resources'])
        geometry=e['centerline']
        for a,b in zip(geometry,geometry[1:]):
            h=math.radians(e['body_heading_deg']) if 'body_heading_deg' in e else math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if e.get('reverse') else 0)
            for x,y,_ in sample([[*a,h],[*b,h]],step=2):
                box=OBB(x+.75*math.cos(h),y+.75*math.sin(h),h,8.344,2.6)
                for key,park in parking.items():
                    if key not in (e['from_node'],e['to_node']) and overlaps(box,park,.1):empty.add(key)
                for key,zone in zones.items():
                    if (e['kind']=='service_access' or key.startswith('access:')) and overlaps(box,zone,.1):required.add(key)
        e['must_be_empty_nodes']=sorted(empty);e['required_resources']=sorted(required)
    for key,node in services.items():
        node['occupation_resources']=['berth:'+key]+(resources(key) if node['kind']!='waiting' else [])
        node['entry_edges']=incoming[key];node['exit_edges']=outgoing[key]
        node['entry_directions_deg']=sorted({round((math.degrees(math.atan2(lookup[e]['centerline'][-1][1]-lookup[e]['centerline'][-2][1],lookup[e]['centerline'][-1][0]-lookup[e]['centerline'][-2][0]))+(180 if lookup[e]['reverse'] else 0))%360,1) for e in incoming[key]})
        node['entry_modes']=sorted({('reverse' if lookup[e]['reverse'] else 'forward') for e in incoming[key]})
        node['entry_motion_directions_deg']=sorted({round(math.degrees(math.atan2(lookup[e]['centerline'][-1][1]-lookup[e]['centerline'][-2][1],lookup[e]['centerline'][-1][0]-lookup[e]['centerline'][-2][0]))%360,1) for e in incoming[key]})
    # Trace access-only alternatives back to the directed road boundary.
    # Road ports are boundaries: never infer a new road turn through a bay.
    port_nodes={p['node_id'] for p in ports}
    def access_ports(site, direction):
        adjacency=outgoing if direction=='exit' else incoming
        found=set();todo=[site];seen={site}
        while todo:
            current=todo.pop()
            for key in adjacency[current]:
                e=lookup[key]
                if e['kind']!='service_access':continue
                nxt=e['to_node'] if direction=='exit' else e['from_node']
                if nxt in port_nodes:found.add(nxt)
                elif nxt not in seen:seen.add(nxt);todo.append(nxt)
        return [p for p in ports if p['node_id'] in found and p['direction']==direction]
    for key,node in services.items():
        node['entry_road_ports']=access_ports(key,'entry')
        node['exit_road_ports']=access_ports(key,'exit')
    raw=dict(schema_version=1,coordinate_frame='UE absolute metres; geometry uses rear axle',
        nodes=list(nodes.values()),edges=edges,movements=movements,service_nodes=services,road_ports=ports,
        source_road_edge_parts=replacement,access_sequences=sequence_records,phase_joins=joins,
        shared_queue_rules=waiting['queue_rules'],access_resource_specs=access_resource_specs,retired_nodes=proposal['retired_waiting'],
        source_sha256=inputs,map_sha256=approval.get('map_sha256',waiting['map_sha256']),routing_enabled=False,
        access_approval=approval_file if approval else None,
        scope='Planning inventory, not automatic dispatch or blanket UE physical acceptance')
    NETWORK.write_text(json.dumps(raw,ensure_ascii=False,indent=2)+'\n')
    load_map(NETWORK)  # Endpoint and movement integrity must hold.
    network=ServiceNetwork(NETWORK)
    shortest={(a,b):network.route(a,b,allow_pending_access=True) for a in services for b in services if a!=b}
    strict={(a,b):network.route(a,b) for a,b in shortest}
    pairs=[]
    for (a,b),best in shortest.items():
        variants=[]
        for first in outgoing[a]:
            for last in incoming[b]:
                route=network.route(a,b,first_edge=first,last_edge=last,allow_pending_access=True)
                if route:variants.append(dict(departure_edge=first,arrival_edge=last,**route))
        corridor=network.corridor(a,b)
        pairs.append(dict(origin=a,destination=b,topologically_reachable=best is not None,
            reachable_without_pending_access=strict[(a,b)] is not None,
            route_without_pending_access=strict[(a,b)],
            best_route=best,port_route_alternatives=variants,legal_corridor_edge_ids=corridor,
            legal_original_road_edge_ids=sorted({lookup[k]['source_road_edge'] for k in corridor if 'source_road_edge' in lookup[k]}),
            waiting_options=network.queue_options(a,b,shortest),end_to_end_ue_validated=False))
    catalog=dict(schema_version=1,status='PLANNING_INVENTORY_COMPLETE',service_node_count=len(services),
        ordered_pair_count=len(pairs),reachable_pair_count=sum(p['topologically_reachable'] for p in pairs),
        strict_reachable_pair_count=sum(r is not None for r in strict.values()),
        pending_reverse_access_count=sum(r.get('access_validation_required',False) for r in sequence_records),
        pending_road_port_count=sum(p['attachment_validation_required'] for p in ports),
        graph=str(NETWORK.relative_to(ROOT)),source_sha256=inputs,
        graph_sha256=hashlib.sha256(NETWORK.read_bytes()).hexdigest(),routing_enabled=False,
        nodes=services,pairs=pairs,
        scope='All registered directed roads and access alternatives; edge-state reachability represents detours without enumerating infinite loop routes; no claim of 552 physical trip validations')
    CATALOG.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
    render(catalog,raw)
    print(catalog['status'],len(services),'nodes',catalog['reachable_pair_count'],'/',len(pairs),'reachable pairs',len(edges),'edges')


def render(catalog,raw):
    render_map(raw)
    lines=['# UE 装卸与等待节点网络','',
        '24 个独立业务节点：W1～W6 各有装货、卸货节点，Q1～Q12 为等待节点；弃用的 W1/W2-local 不入图。',
        '道路保留原来的有向车道、左右转许可、变道和末端掉头；入口和出口分别建端口，保留多个可行接入方式。每个节点还记录接入原道路编号、入口/出口朝向及接入位置；端口数量可能包含同一方向的不同接入位置。车辆朝向按 UE +X/+Y 定义，路径坐标统一为后轴坐标。', '',
        f"全部 {catalog['ordered_pair_count']} 个有序节点对中，{catalog['reachable_pair_count']} 个有拓扑路径。每对保存入口/出口组合的代表路径、所有可达有向道路集合以及途中等待方案。道路有循环，所有路径无法有限逐条穷举；完整图保存全部道路，走廊集合包含合法绕行，代表路径按距离/速度的静态成本选择。",'',
        '**局部接入证据不等于整段 UE 运输验收，全园区自动调度仍关闭。** 已通过的接入采用实际测试过的平顺后轴轨迹；其余短边界连接仍是拓扑关系，执行整段路线时需衔接轨迹。静态时间不含加减速、服务和排队。','',
        (f"9 个平顺边界端口与 W3 装货倒车进入、W4 卸货倒车退出、Q9 倒车进入、Q3 倒车退出这 4 个方案均通过 UE 低速验证；按 5 车、3 车、单车分组，见 [接入验收](ue_access_connections_acceptance.md)。默认排除待验证接入后，{catalog['strict_reachable_pair_count']} 个节点对有路径。仍待验证的倒车序列为 {catalog['pending_reverse_access_count']}，边界端口为 {catalog['pending_road_port_count']}。这表示局部接入缺口已补齐，不表示 552 个节点对逐对完成 UE 运输。" if raw.get('access_approval') else f"默认排除待验证接入时，{catalog['strict_reachable_pair_count']} 个节点对有路径；仍待验证的倒车序列为 {catalog['pending_reverse_access_count']}，边界端口为 {catalog['pending_road_port_count']}。"),'',
        '![节点与有向道路示意图](ue_service_network.svg)','',
        '| 节点 | 类型 | 位置 UE m | 道路入口/出口数 | 入位方式 |','|---|---|---|---:|---|']
    for key,n in catalog['nodes'].items():lines.append(f"| {key} | {n['kind']} | {n['xy_m']} | {len(n['entry_road_ports'])}/{len(n['exit_road_ports'])} | {', '.join(n['entry_modes'])} |")
    lines+=['','## 途中排队规则','',
        '每个等待位容量为 1。候选等待位必须有有向的“源点→等待位→目的点”路径；每项记录两段具体道路、额外距离、是否属于目的仓库原推荐位、所需预约、必须清空的其他点位以及边界连接是否仍待验证。原仓库推荐位是偏好，不会把其他有合法路线的等待位排除。',
        '若进入等待位会经过目的泊位、扫过其停车包络，或申请目的点作业期间占用的共享通道，则该等待方案不能在目的泊位被占用时直接补充排队，单独标为 `requires_destination_clear`。不能仅按地图距离选择等待位。',
        'Q1 出位需 Q2 清空；Q1/Q2 与 W5 共用通道。Q6～Q8、Q9/Q10、Q11/Q12 进出各有共享预约。W5/W6 作业出口连同局部对向通过区预约，直路上可以并行；现场容量、预约 FIFO 和车尾释放仍由后续调度器执行。道路或装卸通道扫过其他停车包络时，边记录 `must_be_empty_nodes`。', '',
        '## 数据与查询','',
        '- `config/topology/industrialarea_ue_service_network.json`：完整有向道路、端口、装卸/等待节点、许可转移、占用约束、来源校验值。',
        '- `config/planning/ue_service_connectivity.json`：全部有序节点对、端口替代路径、可达道路全集及排队等待方案。',
        '- `ServiceNetwork.route()`：可按首/末接入边、被占用节点和是否允许待验证连接筛选路线；`queue_options()` 给出可达且可继续前往目的点的等待位。', '',
        '```bash','.venv/bin/python scripts/build_ue_service_network.py',
        '.venv/bin/python scripts/query_ue_service_network.py W5-loading W6-unloading',
        '.venv/bin/python scripts/query_ue_service_network.py W5-loading W3-loading --include-pending',
        '```','',
        '```python','from logistics_park.routing.service_network import ServiceNetwork',
        "network = ServiceNetwork('config/topology/industrialarea_ue_service_network.json')",
        "route = network.route('W5-loading', 'W6-unloading', allow_pending_access=False)",
        "queues = network.queue_options('W5-loading', 'W6-unloading')",'```','',
        '下一步按具体任务筛选路线，先覆盖六仓库代表性运输，再把等待位容量与作业/路口预约接入路径规划；新入口方案仍需补入证据后发布。']
    (ROOT/'docs/ue_service_network.md').write_text('\n'.join(lines)+'\n')


def render_map(raw):
    """A metric vector overview; directions and permissions remain in the graph."""
    points=[p for e in raw['edges'] for p in e['centerline']]
    xmin,xmax=min(p[0] for p in points),max(p[0] for p in points)
    ymin,ymax=min(p[1] for p in points),max(p[1] for p in points)
    scale=min(800/(xmax-xmin),1180/(ymax-ymin))
    def xy(p):return (100+(p[0]-xmin)*scale,80+(ymax-p[1])*scale)
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1400" viewBox="0 0 1000 1400">',
         '<rect width="1000" height="1400" fill="#fafafa"/>',
         '<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#87939c"/></marker></defs>',
         '<g font-family="sans-serif"><text x="40" y="32" font-size="20">UE service network — +Y ↑ / +X →</text>',
         '<text x="40" y="55" font-size="13">Directed roads; L = loading, U = unloading, Q = waiting. Schematic, not physical trip acceptance.</text>']
    for e in raw['edges']:
        coords=' '.join(f'{x:.2f},{y:.2f}' for x,y in map(xy,e['centerline']))
        pending=e.get('attachment_validation_required') or e.get('access_validation_required')
        color='#d64747' if pending else '#87939c' if e['kind']!='service_access' else '#a2b5bf'
        dash=' stroke-dasharray="5 4"' if pending else ''
        arrow=' marker-end="url(#arrow)"' if e['kind']!='service_access' else ''
        svg.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="1.4"{dash}{arrow}/>')
    colors={'loading':'#16894d','unloading':'#d4770d','waiting':'#196fbb'}
    for key,n in raw['service_nodes'].items():
        x,y=xy(n['xy_m']);label=key.replace('-loading','-L').replace('-unloading','-U')
        west=n.get('warehouse') in ('W3','W4');tx=x-9 if west else x+9;anchor='end' if west else 'start'
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="{colors[n["kind"]]}" stroke="white"/>')
        svg.append(f'<text x="{tx:.2f}" y="{y-7:.2f}" text-anchor="{anchor}" font-size="13" fill="{colors[n["kind"]]}" stroke="#fafafa" stroke-width="3" paint-order="stroke">{label}</text>')
    svg+=['<text x="40" y="1320" font-size="14">Green: loading | Orange: unloading | Blue: waiting | Dashed red: access validation pending</text>',
          '<text x="40" y="1345" font-size="14">Each service/waiting node has capacity 1. Retired W1/W2-local waits are excluded.</text>',
          '<text x="40" y="1370" font-size="14">Road turns, multiple access ports and queue exclusions: ue_service_network.md + JSON catalog.</text></g></svg>']
    (ROOT/'docs/ue_service_network.svg').write_text('\n'.join(svg)+'\n')


if __name__=='__main__':build()
