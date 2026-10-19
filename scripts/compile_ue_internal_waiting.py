#!/usr/bin/env python3
"""Attach internal bays to directed roads; keep new access pending UE tests."""
import copy,json,math,sys
from collections import defaultdict
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.geometry import Polyline

def hermite(a,b,ha,hb):
    distance=math.dist(a,b);count=max(2,math.ceil(distance/.15));rows=[]
    ta=[distance*math.cos(ha),distance*math.sin(ha)];tb=[distance*math.cos(hb),distance*math.sin(hb)]
    for i in range(count+1):
        t=i/count
        rows.append([(2*t**3-3*t*t+1)*a[j]+(t**3-2*t*t+t)*ta[j]+(-2*t**3+3*t*t)*b[j]+(t**3-t*t)*tb[j] for j in (0,1)])
    return rows

def build():
    net=json.loads((ROOT/'config/topology/industrialarea_ue_internal_waiting_candidate_network.json').read_text())
    model=json.loads((ROOT/'config/topology/industrialarea_ue_flow_optimized_transport.json').read_text())
    rows={e['edge_id']:e for e in net['edges']};nodes={n['node_id']:n for n in net['nodes']};cuts=defaultdict(list)
    bays=[n for n in net['service_nodes'] if n.startswith('Q') and int(n[1:])>=26]
    successors=defaultdict(list);predecessors=defaultdict(list)
    for m in net['movements']:
        successors[m['incoming_edge']].append(m['outgoing_edge'])
        predecessors[m['outgoing_edge']].append(m['incoming_edge'])
    def reachable(seeds,links):
        seen=set(seeds);todo=list(seeds)
        while todo:
            for key in links[todo.pop()]:
                if key not in seen and key not in {'access_133','access_134','access_167'}:
                    seen.add(key);todo.append(key)
        return seen
    core=reachable([k for k,e in rows.items() if e['from_node']=='W1-loading'],successors)
    core &= reachable([k for k,e in rows.items() if e['to_node']=='W1-loading'],predecessors)
    for bay in bays:
        entry=rows[bay+'_entry'];points=entry['centerline'];rev=entry.get('reverse',False)
        h=math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0])+(math.pi if rev else 0)
        options=[]
        for key,e in rows.items():
            if e['kind']!='road' or not key.startswith('road_') or key not in core:continue
            line=Polyline(tuple(map(tuple,e['centerline'])));p=line.project(*points[0])
            angle=abs((p.heading-h+math.pi)%(2*math.pi)-math.pi)
            if angle>=math.radians(45):continue
            full=Polyline(tuple(map(tuple,model['physical_centerlines'][e['capacity_alias']])))
            q=full.project(*points[0])
            if q.distance>4:s=max(.05,min(full.length-.05,q.s))
            else:s=min(full.length-.05,q.s+12) if rev else max(.05,q.s-12)
            anchor,rh=full.at(s);mh=rh+(math.pi if rev else 0)
            along=(points[0][0]-anchor[0])*math.cos(mh)+(points[0][1]-anchor[1])*math.sin(mh)
            if along>1.5 and math.dist(anchor,points[0])<40:
                options.append((math.dist(anchor,points[0]),key,anchor,rh))
        if not options:raise ValueError('No directed approach with braking room for '+bay)
        gap,key,anchor,road_heading=min(options)
        lane=rows[key]
        # Use the existing split edge containing this exact directed road port.
        candidates=[]
        for k,e in rows.items():
            if e['kind']=='road' and k.startswith('road_') and k in core and e['capacity_alias']==lane['capacity_alias']:
                line=Polyline(tuple(map(tuple,e['centerline'])));q=line.project(*anchor)
                candidates.append((q.distance,k,q.s))
        error,k,station=min(candidates)
        if error>1e-5:raise ValueError('Port outside existing road '+bay)
        port=bay+'_road_port';nodes[port]=dict(node_id=port,node_type='WAYPOINT',position=list(anchor));cuts[k].append((station,port))
        motion_heading=road_heading+(math.pi if rev else 0)
        end_heading=math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0])
        joined=hermite(anchor,points[0],motion_heading,end_heading)+points[1:]
        entry.update(from_node=port,centerline=joined)
        # Returning along the admitted ingress avoids dead-end gate exits and
        # keeps the same neighbouring-bay clearance in both directions.
        exit=rows[bay+'_exit'];exit.update(to_node=port,centerline=list(reversed(joined)),reverse=not rev)
        for e in (entry,exit):
            e['length_m']=sum(math.dist(a,b) for a,b in zip(e['centerline'],e['centerline'][1:]));e['estimated_travel_time_s']=e['length_m']/e['speed_limit']
        net['service_nodes'][bay].update(entry_road_ports=[dict(node_id=port,source_road_edge=k,road_reference_rear_xy_m=list(anchor),attachment_gap_m=0)],exit_road_ports=[dict(node_id=port,source_road_edge=k,road_reference_rear_xy_m=list(anchor),attachment_gap_m=0)])
    replacements={};edges=[]
    for key,e in rows.items():
        if key not in cuts:edges.append(e);replacements[key]=[key];continue
        line=Polyline(tuple(map(tuple,e['centerline'])));stations=[(0.,e['from_node'])]+sorted(cuts[key])+[(line.length,e['to_node'])]
        pieces=[]
        for i,((lo,frm),(hi,to)) in enumerate(zip(stations,stations[1:])):
            if hi-lo<1e-6:
                # Multiple bays may share an exact road port.
                for row in rows.values():
                    if row['from_node']==to:row['from_node']=frm
                    if row['to_node']==to:row['to_node']=frm
                continue
            cumulative=[0.]
            for a,b in zip(line.points,line.points[1:]):cumulative.append(cumulative[-1]+math.dist(a,b))
            pts=[list(line.at(lo)[0])]+[list(p) for p,s in zip(line.points,cumulative) if lo+1e-7<s<hi-1e-7]+[list(line.at(hi)[0])]
            row=copy.deepcopy(e);eid=key+'__internal_'+str(i);row.update(edge_id=eid,from_node=frm,to_node=to,centerline=pts,length_m=hi-lo,estimated_travel_time_s=(hi-lo)/e['speed_limit']);edges.append(row);pieces.append(eid)
        replacements[key]=pieces
    movements=[]
    for m in net['movements']:
        a=replacements[m['incoming_edge']][-1];b=replacements[m['outgoing_edge']][0]
        row=copy.deepcopy(m);row.update(incoming_edge=a,outgoing_edge=b,movement_id=a+'__'+b);movements.append(row)
    byid={e['edge_id']:e for e in edges}
    pairs={(m['incoming_edge'],m['outgoing_edge']) for m in movements}
    outgoing=defaultdict(list)
    for e in edges:outgoing[e['from_node']].append(e)
    for a in edges:
        for b in outgoing[a['to_node']]:
            is_split=a['edge_id'].split('__internal_')[0]==b['edge_id'].split('__internal_')[0]
            is_new=a.get('site') in bays or b.get('site') in bays
            if not (is_split or is_new) or (a['edge_id'],b['edge_id']) in pairs:continue
            movements.append(dict(movement_id=a['edge_id']+'__'+b['edge_id'],incoming_edge=a['edge_id'],outgoing_edge=b['edge_id'],maneuver='STRAIGHT',reference_path=b['centerline'],conflict_zones=[],speed_limit=min(a['speed_limit'],b['speed_limit'])))
            pairs.add((a['edge_id'],b['edge_id']))
    net.update(nodes=list(nodes.values()),edges=edges,movements=movements,scope=f'{sum(s["kind"]=="waiting" for s in net["service_nodes"].values())} inside-park waiting bays; new full road connections pending validation')
    dest=ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json';dest.write_text(json.dumps(net,indent=2)+'\n')
    # Recompile the exact legal network into the established <=5 m geometry.
    parts=[];part_ids={};tnodes={n['node_id']:n for n in nodes.values()}
    for e in edges:
        line=Polyline(tuple(map(tuple,e['centerline'])));count=max(1,math.ceil(line.length/5));ids=[];stations=[0.]
        for a,b in zip(line.points,line.points[1:]):stations.append(stations[-1]+math.dist(a,b))
        for i in range(count):
            lo,hi=line.length*i/count,line.length*(i+1)/count;key=e['edge_id']+'__test'+str(i)
            frm=e['from_node'] if i==0 else e['edge_id']+'__point'+str(i);to=e['to_node'] if i==count-1 else e['edge_id']+'__point'+str(i+1)
            pts=[list(line.at(lo)[0])]+[list(p) for p,s in zip(line.points,stations) if lo+1e-7<s<hi-1e-7]+[list(line.at(hi)[0])]
            row=copy.deepcopy(e);row.update(edge_id=key,from_node=frm,to_node=to,centerline=pts,speed_limit=8 if e['kind']=='road' else e['speed_limit'],resource=e.get('resource') if e.get('resource') in model['control_zones'] else None);parts.append(row);ids.append(key)
            if e.get('straight_until_m'):row['straight_until_m']=max(0,e['straight_until_m']-lo)
            tnodes[frm]=dict(node_id=frm,node_type='WAYPOINT',position=pts[0]);tnodes[to]=dict(node_id=to,node_type='WAYPOINT',position=pts[-1])
        part_ids[e['edge_id']]=ids
        model['physical_centerlines'].setdefault(e['capacity_alias'],e['centerline']);model['physical_capacities'].setdefault(e['capacity_alias'],1)
        if e.get('site') in bays:model['physical_centerlines'][e['capacity_alias']]=e['centerline']
    tm=[];part_lookup={e['edge_id']:e for e in parts}
    def movement(a,b):
        tm.append(dict(movement_id=a+'__'+b,incoming_edge=a,outgoing_edge=b,maneuver='STRAIGHT',reference_path=part_lookup[b]['centerline'],conflict_zones=[],speed_limit=min(part_lookup[a]['speed_limit'],part_lookup[b]['speed_limit'])))
    for ids in part_ids.values():
        for a,b in zip(ids,ids[1:]):movement(a,b)
    for m in movements:movement(part_ids[m['incoming_edge']][-1],part_ids[m['outgoing_edge']][0])
    model.update(nodes=list(tnodes.values()),edges=parts,movements=tm)
    for spec in model.get('shared_resources',{}).values():
        spec['edge_ids']=[p['edge_id'] for p in parts if any(p['edge_id'].split('__test')[0].split('__internal_')[0]==k.split('__test')[0] for k in spec['edge_ids'])]
    out=ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json';out.write_text(json.dumps(model,indent=2)+'\n')
    from build_ue_waiting_scan import build as scan
    scan(source=dest,expected_sites=bays,output_stem='industrialarea_ue_internal_waiting_connected_scan',allow_candidate=True)
    print('Compiled',len(bays),'inside bays with exact directed road joins')
if __name__=='__main__':build()
