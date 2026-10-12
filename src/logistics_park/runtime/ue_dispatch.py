"""UE dispatch application: existing FCFS/capacity/guards, interchangeable adapter messages."""
import math
import json
import time
from logistics_park.common.messages import VehicleState,ControlCommand
from logistics_park.scheduling.road_capacity import RoadCapacity
from logistics_park.scheduling.fcfs import FCFSScheduler,EntranceRequest
from logistics_park.safety.following_guard import following_speed_cap
from .geometry import project,at_station,support,euclidean_xy
from .speed_limits import SpeedLimits,bend_stations
WHEELBASE_M=2.3285


def run_dispatch(adapter,graph,survey,scenario,observer,output):
    routes=scenario['routes'];NAMES=tuple(routes)
    limits=SpeedLimits(**scenario.get('speed_limits',{}))
    bends={}
    for n,r in routes.items():
        stations=[0.]
        for a,b in zip(r['path'],r['path'][1:]):stations.append(stations[-1]+euclidean_xy(a,b))
        bends[n]=bend_stations(r['path'],stations)
    UP,DOWN=scenario['junctions'];LINK=scenario['link_edge'];js=survey['junctions']
    edges={key:{'centerline':edge.centerline.points} for key,edge in graph.edges.items()}
    capacity=RoadCapacity(graph);scheduler=FCFSScheduler((UP,DOWN),capacity)
    report={'status':'FAIL','failure':[],'routes':routes,'samples':[],'test_condition':'temporarily hold downstream admission until three trucks queue and upstream truck stands still for 4 seconds; then reopen', 'coordinate_convention':'vehicle-relative right-hand traffic; downstream +Y to +X turns are LEFT','link_capacity':capacity.capacities[LINK]}
    report['speed_limits']=vars(limits)
    requested=set();entered=set();cleared=set();finished=set();released=None;hold_start=None;max_parallel=0;initial_full=False
    waiting_started={};waits={}
    try:
        initial=adapter.read_states()
        if any(euclidean_xy((initial[n].x,initial[n].y),routes[n]['start'])>.2 for n in NAMES):
            raise RuntimeError('vehicles must start at the configured scenario positions')
        baseline={n:adapter.telemetry[n]['collision']['time_stamp'] for n in NAMES}
        print('AirSim Adapter: dispatch running',flush=True)
        sequence=0
        begin=time.monotonic();last_tick=begin;last_print=begin;last_progress=begin;best_progress=0
        while time.monotonic()-begin<scenario.get('max_runtime_s',300):
            now=time.monotonic();t=now-begin;dt=min(.3,max(.01,now-last_tick));last_tick=now
            measured=adapter.read_states()
            poses={};speeds={};audits={};bds={};errors=[];stations={};states={}
            for n in NAMES:
                poses[n]=[measured[n].x,measured[n].y,adapter.telemetry[n]['ue_z_m']];speeds[n]=abs(measured[n].speed)
                a,e=observer.read(n);audits[n]=a;errors += [n+': '+v for v in e];bds[n]={axis:support(a,axis) for axis in (0,1)}
                hit=adapter.telemetry[n]['collision']
                if hit['has_collided'] and hit['time_stamp']!=baseline[n]:errors.append(n+' native collision '+hit['object_name'])
                error,station=project(poses[n][:2],routes[n]['path']);stations[n]=station
                if error>.65:errors.append(n+' path deviation')
                edge=routes[n]['visits'][0]['incoming']
                for v in routes[n]['visits']:
                    j=js[v['junction']];cy=j['center_m'][1];hy=j['half_extent_m'][1]
                    exitaxis=1 if v['outgoing'] in (LINK,'road_j_3_1_gate_north_1_out') else 0
                    exitboundary=j['center_m'][exitaxis]+j['half_extent_m'][exitaxis]
                    if poses[n][exitaxis]>exitboundary:edge=v['outgoing']
                    elif poses[n][1]>cy-hy:edge=v['connector']
                states[n]=VehicleState(n,*poses[n][:2],measured[n].heading,speed=speeds[n],edge_id=edge,s=project(poses[n][:2],edges[edge]["centerline"])[1])
            capacity.update(states);full=capacity.road_state(LINK).is_saturated
            if released is None and len(capacity.occupants[LINK])==capacity.capacities[LINK]:initial_full=True
            for n in NAMES:
                for v in routes[n]['visits']:
                    key=(n,v['junction']);j=js[v['junction']];entry=j['center_m'][1]-j['half_extent_m'][1]
                    if key in cleared:continue
                    if v['junction']==DOWN and n==NAMES[0] and (n,UP) not in cleared:continue
                    if key not in requested and entry-bds[n][1][1]<=2:
                        scheduler.request(EntranceRequest(n,v['junction'],v['incoming'],v['connector'],v['outgoing'],t,max(0,states[n].s)));requested.add(key)
                    # Any body box overlap with the physical junction footprint
                    # requires its matching permission, including the trailer tail.
                    inside=all(bds[n][a][1]>j['center_m'][a]-j['half_extent_m'][a]+.05 and bds[n][a][0]<j['center_m'][a]+j['half_extent_m'][a] for a in (0,1))
                    if inside:
                        if not scheduler.allows(n,v['junction'],v['connector']):errors.append(n+' unauthorized entry '+v['junction'])
                        elif key not in entered:scheduler.mark_entered(v['junction'],n,t);entered.add(key)
                    axis=1 if v['outgoing'] in (LINK,'road_j_3_1_gate_north_1_out') else 0
                    exit=j['center_m'][axis]+j['half_extent_m'][axis]
                    if key in entered and bds[n][axis][0]>exit+.3:
                        scheduler.complete(v['junction'],n,t);cleared.add(key)
            if released is None and (NAMES[0],UP) in requested and speeds[NAMES[0]]<.01:
                hold_start=hold_start if hold_start is not None else t
                if t-hold_start>=4:
                    if not initial_full or (NAMES[0],UP) in entered:errors.append('failed to demonstrate upstream backpressure')
                    released=t;report['reopened_at_s']=t;report['upstream_stationary_hold_before_reopen_s']=t-hold_start;print('downstream reopened after upstream stationary hold',flush=True)
            elif released is None:hold_start=None
            def can_enter(edge,n):
                if edge!=LINK and released is None:return False
                v=next(v for v in routes[n]['visits'] if v['outgoing']==edge)
                j=js[v['junction']];axis=1 if edge in (LINK,'road_j_3_1_gate_north_1_out') else 0;boundary=j['center_m'][axis]+j['half_extent_m'][axis]
                return all(bds[m][axis][0]-boundary>=11.344 for m in NAMES if m!=n and states[m].edge_id==edge)
            scheduler.advance(t,can_enter=can_enter);max_parallel=max(max_parallel,len(scheduler.owners))
            for jkey in (UP,DOWN):
                occupants=[n for n,j in entered-cleared if j==jkey]
                if len(occupants)>1:errors.append('multiple owners inside '+jkey)
            if capacity.violations:errors.append('capacity violation')
            if errors:raise RuntimeError('; '.join(errors))
            for n in NAMES:
                r=routes[n];nextv=next((v for v in r['visits'] if (n,v['junction']) not in cleared),None)
                target=limits.straight_m_s
                if released is None and n!=NAMES[0]:target=0
                if nextv and not scheduler.allows(n,nextv['junction'],nextv['connector']):
                    j=js[nextv['junction']];distance=max(0,j['center_m'][1]-j['half_extent_m'][1]-.8-bds[n][1][1]);target=min(target,math.sqrt(1.4*distance),distance/1.2)
                    key=(n,nextv['junction'])
                    if key in requested and speeds[n]<.01:
                        waiting_started.setdefault(key,t);waits[str(key)]=max(waits.get(str(key),0),t-waiting_started[key])
                    else:waiting_started.pop(key,None)
                # Same-edge FIFO uses path arc length on a common geometric lane.
                ahead=[]
                for m in NAMES:
                    if m==n:continue
                    lateral,leader_s=project(poses[m][:2],r['path'])
                    if lateral<.65 and leader_s>stations[n]+.1:ahead.append((leader_s,m))
                if ahead:
                    leader_s,m=min(ahead)
                    common='diagnostic_path_'+n
                    rear=VehicleState(n,*poses[n][:2],states[n].heading,speed=speeds[n],edge_id=common,s=stations[n])
                    front=VehicleState(m,*poses[m][:2],states[m].heading,speed=speeds[m],edge_id=common,s=leader_s)
                    target=min(target,following_speed_cap(rear,front,8.344,standstill_gap=3,time_headway=1.5,deceleration=1,reaction_time=.2))
                distance=euclidean_xy(poses[n],r['goal']);allclear=all((n,v['junction']) in cleared for v in r['visits'])
                if allclear:target=min(target,.6*distance)
                if allclear and distance<.8:
                    target=0
                    if speeds[n]<.01:finished.add(n)
                yaw=states[n].heading;rear=[poses[n][0]-.5*WHEELBASE_M*math.cos(yaw),poses[n][1]-.5*WHEELBASE_M*math.sin(yaw)];_,station=project(rear,r['path']);aim=at_station(r['path'],station+max(2.7,speeds[n]*1.3))
                alpha=(math.atan2(aim[1]-rear[1],aim[0]-rear[0])-yaw+math.pi)%(2*math.pi)-math.pi;delta=math.atan2(2*WHEELBASE_M*math.sin(alpha),max(.5,euclidean_xy(aim,rear)));steer=max(-1,min(1,delta/math.radians(40)))
                if abs(steer)>.15:target=min(target,limits.turn_m_s)
                upcoming=next((s for s in bends[n] if s>=station-.3),None)
                if upcoming is not None:target=min(target,limits.approach(limits.turn_m_s,upcoming-station))
                omega=target*math.tan(steer*adapter.config.max_steer_rad)/WHEELBASE_M
                adapter.send(ControlCommand(n,sequence,adapter.clock(),target,omega,ttl=adapter.config.command_ttl_s))
            sequence+=1
            report['samples'].append({'t_s':t,'positions':poses,'speeds_m_s':speeds,'owners':{j:g.request.vehicle_id for j,g in scheduler.owners.items()},'link_occupants':sorted(capacity.occupants[LINK]),'link_saturated':full,'body_audits':audits})
            if errors:raise RuntimeError('; '.join(errors))
            progress=sum(stations.values())
            if progress>best_progress+.5:best_progress=progress;last_progress=now
            if released is not None and now-last_progress>35 and len(finished)<4:raise RuntimeError('recovery stalled for 35 seconds')
            if now-last_print>5:
                print(f'four trucks: owners={report["samples"][-1]["owners"]}, queued link={len(capacity.occupants[LINK])}, cleared={len(cleared)}/5, finished={len(finished)}/4',flush=True);last_print=now
            if len(finished)==4:break
            time.sleep(.1)
        else:raise RuntimeError('four-truck test timeout')
        order=[e['vehicle_id'] for e in scheduler.events if e['event']=='ENTERED' and e['intersection_id']==DOWN]
        if order!=scenario['expected_downstream_order']:raise RuntimeError('downstream FIFO violated')
        if len(cleared)!=5 or max_parallel<2:raise RuntimeError('missing completion or parallel-junction evidence')
        report['status']='PASS'
    except Exception as e:
        report['failure'].append(repr(e));adapter.fail_stop('dispatch: '+str(e));print('FAIL',e,flush=True)
    finally:
        adapter.close()
        report.update(events=scheduler.events,cleared=[list(k) for k in sorted(cleared)],finished=sorted(finished),maximum_parallel_junctions=max_parallel,waiting_s=waits,initial_link_saturation_observed=initial_full)
        report['adapter_closed']=adapter.closed
        report['adapter_fault']=adapter.fault
        report['close_result']=adapter.close_result
        if adapter.fault or not adapter.close_result.get('api_restored'):report['status']='FAIL'
        output.mkdir(parents=True,exist_ok=True)
        (output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
