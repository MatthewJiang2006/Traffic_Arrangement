#!/usr/bin/env python3
"""Audit recorded UE poses, arrivals and control constraints without replaying physics."""
import argparse, bisect, json, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def angle(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def audit(directory):
    report = json.loads((directory / 'physical_acceptance.json').read_text())
    network = json.loads((ROOT / 'config/topology/industrialarea_ue_internal_waiting_network.json').read_text())
    model = json.loads((ROOT / 'config/topology/industrialarea_ue_internal_waiting_transport.json').read_text())
    edges = {e['edge_id']: e for e in model['edges']}
    frames = [json.loads(line) for line in (directory / 'physical_trajectory.jsonl').open()]
    times = [f['t_s'] for f in frames]
    arrivals = []
    for event in report['road_events']:
        if event['event'] != 'ARRIVED' or event['site'] not in network['service_nodes']:
            continue
        index = min(len(frames)-1, bisect.bisect_left(times, event['time']))
        site = network['service_nodes'][event['site']]
        x, y, yaw, speed = frames[index]['positions'][event['vehicle_id']]
        arrivals.append(dict(time_s=event['time'], vehicle=event['vehicle_id'], site=event['site'],
            position_error_m=math.dist((x,y), site['rear_xy_m']),
            heading_error_deg=math.degrees(angle(yaw, math.radians(site['heading_deg']))), speed_m_s=speed))
    # Only test the lane envelope when the complete body is on a straight
    # physical road. A nose crossing into an intersection is not a lane exit.
    road_samples = 0; violations = []; transitions = []; delayed = []
    for line in (directory / 'physical_control_diagnostics.jsonl').open():
        row = json.loads(line)
        for name, detail in row['execution_details'].items():
            edge = edges.get(detail['edge_id'])
            if not edge or edge['kind'] != 'road':
                continue
            points = model['physical_centerlines'].get(edge['capacity_alias'])
            if not points or len(points) != 2:
                continue
            x,y,yaw,speed = row['positions'][name]
            (ax,ay),(bx,by) = points; length = math.hypot(bx-ax,by-ay)
            co,si = (bx-ax)/length,(by-ay)/length
            heading = math.atan2(si,co); diff = angle(yaw,heading)
            cx,cy = x+.75*math.cos(yaw),y+.75*math.sin(yaw)
            longitudinal = (cx-ax)*co+(cy-ay)*si
            lateral = abs(-(cx-ax)*si+(cy-ay)*co)
            along = 4.172*abs(math.cos(diff))+1.3*abs(math.sin(diff))
            extent = 4.172*abs(math.sin(diff))+1.3*abs(math.cos(diff))
            if longitudinal < along+.25 or longitudinal > length-along-.25:
                continue
            road_samples += 1
            excess = lateral+extent-edge['width']/2
            if excess > .05:
                flag=dict(time_s=row['t_s'], vehicle=name, edge=edge['edge_id'],
                    lane_envelope_excess_m=excess, axle_lateral_m=lateral,
                    heading_error_deg=math.degrees(diff))
                # The physical road spans the access attachment. Classify a
                # truck whose tail still occupies that curved access as a
                # merge transition, rather than a wholly on-road vehicle.
                route=detail['route_edges']; index=route.index(detail['edge_id'])
                begin=index
                while begin>0 and edges.get(route[begin-1],{}).get('kind')=='road':begin-=1
                parts=[edges.get(k) for k in route[:begin]]
                if all(parts):
                    start=sum(sum(math.dist(a,b) for a,b in zip(e['centerline'],e['centerline'][1:])) for e in parts)
                    transition=detail['progress_m']-start < 4.172-.75+.25
                else:
                    transition=None
                flag['tail_in_access_transition']=transition
                (transitions if transition else violations).append(flag)
            trace = row.get('control_trace',{}).get(name,[])
            for before,after in zip(trace,trace[1:]):
                if after['stage']=='following' and before['target_m_s'] > .3 and after['target_m_s'] < .08:
                    delayed.append(dict(time_s=row['t_s'],vehicle=name,stage='following',
                        prior_target_m_s=before['target_m_s']))
    final = frames[-1]['positions']; parking=[]
    for name, node in report['parked_nodes'].items():
        site=network['service_nodes'][node]; x,y,yaw,speed=final[name]
        parking.append(dict(vehicle=name,node=node,position_error_m=math.dist((x,y),site['rear_xy_m']),
            heading_error_deg=math.degrees(angle(yaw,math.radians(site['heading_deg']))),speed_m_s=speed))
    states={};active={};spans=[]
    for line in (directory/'physical_adapter.jsonl').open():
        row=json.loads(line);now=row['monotonic_s']
        if row['event']=='STATE':states=row['vehicles']
        if row['event']!='COMMAND':continue
        name=row['vehicle']
        if abs(row['v_target'])>.3 and abs(states.get(name,{}).get('speed',0))<.08:
            active.setdefault(name,[now,now])[1]=now
        elif name in active:
            begin,end=active.pop(name);spans.append(dict(vehicle=name,duration_s=end-begin))
    return dict(source=str(directory),tasks_completed=report['completed_tasks'],
        vehicle_collisions=report['vehicle_collisions'],body_obstacle_contacts=report['body_obstacle_contacts'],
        ground_support_contact_samples=len(report['ground_contact_samples']),
        local_stall=report['local_progress_check'],parking=parking,arrivals=arrivals,
        straight_road_body_samples=road_samples,lane_envelope_violations=violations,
        access_merge_envelope_flags=transitions,
        maximum_continuous_start_response_s=max((s['duration_s'] for s in spans),default=0),
        following_stop_samples=delayed,
        limitations=['Lane envelope uses recorded topology, not wheel raycasts at every frame.',
            'Following stops require checking the leader and other constraints before classifying delay.'])


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); result=audit(args.run.resolve())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('parking','arrivals','lane_envelope_violations','access_merge_envelope_flags','following_stop_samples')},ensure_ascii=False))
    print('parking > 5 degrees:',sum(abs(p['heading_error_deg'])>5 for p in result['parking']))
    print('lane envelope flags:',len(result['lane_envelope_violations']))
