#!/usr/bin/env python3
"""Derive a routable transport map without changing the surveyed base map."""
import copy
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.turn_geometry import right_turn_path
from logistics_park.map.geometry import Polyline
from logistics_park.map.map_loader import load_map
from ue_uturn_paths import arc,WHEELBASE


def build():
    base=json.loads((ROOT/'config/topology/industrialarea_ue.json').read_text())
    survey=json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text())
    sites=copy.deepcopy(json.loads((ROOT/'config/traffic/industrialarea_ue_test_sites.json').read_text())['sites'])
    paths=json.loads((ROOT/'config/traffic/industrialarea_ue_uturn_paths.json').read_text())['paths']
    nodes={n['node_id']:copy.deepcopy(n) for n in base['nodes']}
    original={e['edge_id']:copy.deepcopy(e) for e in base['edges']}
    for e in original.values():
        if e.get('maneuver')=='RIGHT':
            e['centerline']=right_turn_path(e)
            e['straight_until_m']=math.dist(*e['centerline'][:2])
            e['right_turn_policy']='delayed_yaw_lane_dependent_radius'
            e['speed_limit']=min(e['speed_limit'],1.5)
            e['maximum_path_error_m']=1.2
    cuts={k:[] for k in original if k.startswith('road_')}
    extra=[];zones={}
    for j,s in survey['junctions'].items():
        cx,cy=s['center_m'];hx,hy=s['half_extent_m']
        zones[j]={'bounds':[cx-hx,cy-hy,cx+hx,cy+hy],'kind':'junction'}
    zones['j_0_0']['bounds'][1]=49.3973
    zones['j_3_0']['bounds'][3]=304.8267
    # The verified short-truck three-arc turn sweeps behind the asphalt
    # junction boundary. Keep approaching trucks outside its manoeuvre buffer.
    for j in ('j_1_2','j_3_2'):zones[j]['bounds'][0]-=7.2

    def cut(edge,position,name):
        poly=Polyline(tuple(map(tuple,original[edge]['centerline'])))
        proj=poly.project(*position);point=poly.at(proj.s)[0]
        if proj.distance>.02 or not .001<proj.s<poly.length-.001:
            raise ValueError('invalid cut '+name)
        for s,n in cuts[edge]:
            if abs(s-proj.s)<1e-6:return n
        nodes[name]={'node_id':name,'node_type':'WAYPOINT','position':list(point)}
        cuts[edge].append((proj.s,name))
        return name

    def connector(key,start,end,path,kind,resource):
        path=copy.deepcopy(path);path[0]=nodes[start]['position'];path[-1]=nodes[end]['position']
        extra.append({'edge_id':key,'from_node':start,'to_node':end,'centerline':path,
                      'width':3.8,'speed_limit':1.0 if kind=='uturn' else 1.5,
                      'storage_capacity':1,'kind':kind,'resource':resource,'capacity_alias':key})

    for j,other in [('j_1_2','j_1_1'),('j_3_2','j_3_1')]:
        for key in ('road_'+other+'_'+j,'road_'+j+'_'+other):
            p=original[key]['centerline'][0]
            cut(key,[zones[j]['bounds'][0],p[1]],j+'_buffer_'+('in' if key.endswith(j) else 'out'))

    lane_pairs=[]
    # Every two-lane road retains both straight continuations. These explicit
    # lane changes supply the correct lane before a legally restricted turn.
    for key,left in original.items():
        if not key.startswith('road_') or left.get('lane_side')!='LEFT' or key+'_right' not in original:continue
        lane_pairs.append(key)
        right=original[key+'_right'];poly=Polyline(tuple(map(tuple,left['centerline'])))
        for suffix,source,target,length in [('R_L',key+'_right',key,14.),('L_R',key,key+'_right',18.)]:
            s=(poly.length-length)/2
            pa=Polyline(tuple(map(tuple,original[source]['centerline']))).at(s)[0]
            pb=Polyline(tuple(map(tuple,original[target]['centerline']))).at(s+length)[0]
            a=cut(source,pa,'lc_'+key+'_'+suffix+'_in')
            b=cut(target,pb,'lc_'+key+'_'+suffix+'_out')
            # Quintic lateral shift has zero endpoint slope and curvature.
            psource=Polyline(tuple(map(tuple,original[source]['centerline'])))
            ptarget=Polyline(tuple(map(tuple,original[target]['centerline'])))
            path=[]
            for i in range(81):
                t=i/80;q=10*t**3-15*t**4+6*t**5
                p=psource.at(s+length*t)[0];r=ptarget.at(s+length*t)[0]
                path.append([p[0]+q*(r[0]-p[0]),p[1]+q*(r[1]-p[1])])
            resource='lc_'+key
            if 'gate_south_0' in key:resource='j_0_0'
            elif 'gate_north_0' in key:resource='j_3_0'
            if resource not in zones:
                xs=[p[0] for p in path];ys=[p[1] for p in path]
                # Both directions of lane change share an exclusive lane-pair zone.
                margin=1.4
                bounds=[min(xs)-margin,min(ys)-.5,max(xs)+margin,max(ys)+.5]
                zones[resource]={'bounds':bounds,'kind':'lane_change'}
            else:
                if zones[resource]['kind']=='lane_change':
                    box=zones[resource]['bounds'];xs=[p[0] for p in path];ys=[p[1] for p in path]
                    zones[resource]['bounds']=[min(box[0],min(xs)-1.4),min(box[1],min(ys)-.5),max(box[2],max(xs)+1.4),max(box[3],max(ys)+.5)]
            connector('change_'+key+'_'+suffix,a,b,path,'lane_change',resource)

    # Straight traffic participates in the lane-change zone too. Otherwise a
    # vehicle changing lanes would have permission while another drove through.
    for key in lane_pairs:
        resource='lc_'+key
        if resource not in zones:continue
        box=zones[resource]['bounds']
        for edge in (key,key+'_right'):
            poly=Polyline(tuple(map(tuple,original[edge]['centerline'])))
            for label,y in [('lo',box[1]),('hi',box[3])]:
                x=poly.at(poly.length/2)[0][0]
                cut(edge,[x,y],resource+'_'+label+('_right' if edge.endswith('_right') else ''))

    # Wide-road arrival sections move 5m inward, allowing a forward-only
    # return turn at the actual road end, beyond the arriving vehicle.
    for site in ('A','C'):
        for endpoint in ('start','finish'):sites[site][endpoint]['xy_m'][1]=70. if site=='A' else 284.
    for label,site in sites.items():
        for endpoint in ('start','finish'):
            p=site[endpoint];yaw=math.radians(p['heading_deg'])
            rear=[p['xy_m'][0]-WHEELBASE/2*math.cos(yaw),p['xy_m'][1]-WHEELBASE/2*math.sin(yaw)]
            p['node_id']=cut(p['edge_id'],rear,'site_'+label+'_'+endpoint)
            p['rear_xy_m']=nodes[p['node_id']]['position']
        site['finish_nodes']=[site['finish']['node_id']]
        if label in ('B','D'):
            j=site['uturn_junction'];a=j+'_in_W';b=j+'_out_W'
            path=paths[label]['rear_path'];x=nodes[a]['position'][0]
            # Keep only the junction portion of the verified three-arc turn.
            first=next(i for i,p in enumerate(path) if p[0]>=x)
            last=max(i for i,p in enumerate(path) if p[0]>=x)
            trimmed=[nodes[a]['position']]+path[first:last+1]+[nodes[b]['position']]
            connector('uturn_'+label,a,b,trimmed,'uturn',j)
            extra[-1]['straight_until_m']=max(0,42.9989-5-x)
        else:
            j=site['uturn_junction'];cx=survey['junctions'][j]['center_m'][0]
            north=1 if label=='A' else -1
            # Arrive in -north direction, left turn into the opposite right lane.
            inner_x=cx+north*1.9;out_x=cx-north*5.7
            incoming=site['finish']['edge_id'].removesuffix('_right')
            finish=site['finish']['rear_xy_m'];left_finish=cut(incoming,[inner_x,finish[1]],'site_'+label+'_finish_left')
            site['finish_nodes'].append(left_finish)
            shifted=[inner_x,finish[1]-north*14]
            turn_y=55.5 if label=='A' else 299.
            start=cut(incoming,[inner_x,turn_y],'uturn_'+label+'_in')
            out_edge=site['start']['edge_id']
            end=cut(out_edge,[out_x,turn_y],'uturn_'+label+'_out')
            shifted_node=cut(incoming,shifted,'uturn_'+label+'_shift_end')
            change=[]
            for i in range(81):
                t=i/80;q=10*t**3-15*t**4+6*t**5
                change.append([finish[0]+q*(inner_x-finish[0]),finish[1]-north*14*t])
            connector('change_site_'+label,site['finish']['node_id'],shifted_node,change,'lane_change',j)
            center=[cx-north*1.9,turn_y]
            connector('uturn_'+label,start,end,arc(center,[inner_x,turn_y],[out_x,turn_y],-1,3.8),'uturn',j)
        # Collect completed test vehicles beyond the site's departure point,
        # outside the terminal control zone; no warehouse is invented here.
        p=site['start']
        pos=list(p['rear_xy_m'])
        if label in ('A','C'):pos[1]=156. if label=='A' else 198.
        else:pos[0]-=15
        edge=p['edge_id']
        if label in ('A','C'):
            edge='road_j_1_0_j_2_0_right' if label=='A' else 'road_j_2_0_j_1_0_right'
        site['egress_node']=cut(edge,pos,'site_'+label+'_egress')

    # Split measured roads and retain their physical capacity aliases.
    edges={};parts={};new_nodes=set(nodes)-{n['node_id'] for n in base['nodes']}
    for key,e in original.items():
        kind='junction' if key.startswith('turn_') else 'road'
        resource=None
        if kind=='junction':resource='_'.join(key.split('_')[1:4])
        elif 'gate_south_0' in key:resource='j_0_0'
        elif 'gate_north_0' in key:resource='j_3_0'
        if key not in cuts or not cuts[key]:
            row=copy.deepcopy(e);row.update(kind=kind,resource=resource,capacity_alias=key)
            edges[key]=row;parts[key]=[key];continue
        poly=Polyline(tuple(map(tuple,e['centerline'])))
        stops=[(0,e['from_node'])]+sorted(cuts[key])+[(poly.length,e['to_node'])]
        parts[key]=[]
        for i,((s,a),(t,b)) in enumerate(zip(stops,stops[1:])):
            row=copy.deepcopy(e);new=key+'__p'+str(i)
            row.update(edge_id=new,from_node=a,to_node=b,centerline=[nodes[a]['position'],nodes[b]['position']],kind=kind,resource=resource,capacity_alias=key)
            if resource is None:
                mid=poly.at((s+t)/2)[0]
                for zone,spec in zones.items():
                    x0,y0,x1,y1=spec['bounds']
                    if (spec['kind']=='lane_change' or zone in ('j_1_2','j_3_2')) and x0<=mid[0]<=x1 and y0<=mid[1]<=y1:
                        row['resource']=zone;break
            edges[new]=row;parts[key].append(new)
    for row in extra:edges[row['edge_id']]=row
    movements=[];seen=set()
    def movement(a,b,maneuver='STRAIGHT'):
        if (a,b) in seen:return
        seen.add((a,b));movements.append({'movement_id':a+'__'+b,'incoming_edge':a,'outgoing_edge':b,'maneuver':maneuver,
            'reference_path':[edges[a]['centerline'][-2],edges[a]['centerline'][-1],edges[b]['centerline'][1]],
            'conflict_zones':[],'speed_limit':min(edges[a]['speed_limit'],edges[b]['speed_limit'])})
    for m in base['movements']:movement(parts[m['incoming_edge']][-1],parts[m['outgoing_edge']][0],m['maneuver'])
    incoming={n:[] for n in nodes};outgoing={n:[] for n in nodes}
    for k,e in edges.items():incoming[e['to_node']].append(k);outgoing[e['from_node']].append(k)
    for node in sorted(new_nodes):
        for a in incoming[node]:
            for b in outgoing[node]:movement(a,b,'UTURN' if edges[b]['kind']=='uturn' else 'STRAIGHT')
    for label in ('B','D'):
        u=edges['uturn_'+label]
        for a in incoming[u['from_node']]:movement(a,u['edge_id'],'UTURN')
        for b in outgoing[u['to_node']]:movement(u['edge_id'],b,'UTURN')
    result={'description':'UE transport extension of measured park; explicit legal lane changes and forward U-turns',
            'path_reference':'rear_axle','wheelbase_m':WHEELBASE,'base_map':'config/topology/industrialarea_ue.json',
            'speed_limits':json.loads((ROOT/'config/runtime/ue_speed_limits.json').read_text()),
            'nodes':list(nodes.values()),'edges':list(edges.values()),'movements':movements,'sites':sites,'control_zones':zones,
            'physical_capacities':{k:e['storage_capacity'] for k,e in original.items()}}
    result['physical_centerlines']={k:e['centerline'] for k,e in original.items()}
    for e in extra:
        result['physical_capacities'][e['edge_id']]=1
        result['physical_centerlines'][e['edge_id']]=e['centerline']
    target=ROOT/'config/topology/industrialarea_ue_transport.json';target.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    graph=load_map(target)
    print('Transport map:',len(graph.nodes),'nodes,',len(graph.edges),'edges,',len(graph.movements),'legal movements')
    return result


if __name__=='__main__':build()
