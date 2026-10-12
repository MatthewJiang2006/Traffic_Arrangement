"""Resume a trusted local diagnostic checkpoint; never produce acceptance."""
import argparse,pickle,sys,math,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from simulate_ue_continuous_tasks import cache_routes
from logistics_park.scheduling.task_dispatch import WorkStatus
from logistics_park.runtime.fleet_progress import FleetProgressWatch,task_progress,LocalProgressWatch,local_progress

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('checkpoint',type=Path);parser.add_argument('--seconds',type=float,default=600)
    parser.add_argument('--complete-parking',action='store_true')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:parser.error('--seconds must be finite and positive')
    row=pickle.loads(args.checkpoint.read_bytes());e=row['executor'];states=row['states'];cache_routes(e.dispatch.network)
    if not hasattr(e.controller,'local_exit_bookings'):e.controller.local_exit_bookings={}
    watch=FleetProgressWatch(15.,.25)
    local_watch=LocalProgressWatch(20.,90.) if e.controller.local_traffic else None
    failure=None
    dt=.1;start=row['now'];out=args.checkpoint.parent/'debug_checkpoint.pkl'
    for tick in range(1,int(args.seconds/dt)+1):
        now=start+tick*dt;commands=e.tick(states,now)
        token,deadline=task_progress(e,now)
        progress=watch.observe(states,now,token,scheduled_activity_until=deadline)
        if progress['stalled']:
            out.write_bytes(pickle.dumps(dict(executor=e,states=states,now=now)))
            failure=progress;print('DIAGNOSTIC STOP',round(now),progress,flush=True);break
        if local_watch and tick%10==0:
            local_guard,_=local_progress(e,states,now,local_watch)
            if local_guard['stalled']:
                failure=local_guard;print('DIAGNOSTIC LOCAL STOP',round(now),local_guard,flush=True);break
        for name,s in states.items():
            target,delta=commands.get(name,(0.,0.));v=s.speed+max(-.6*dt,min(.6*dt,target-s.speed))
            heading=s.heading+.5*v*math.tan(delta)/2.3285*dt
            s.x+=v*math.cos(heading)*dt;s.y+=v*math.sin(heading)*dt
            s.heading+=v*math.tan(delta)/2.3285*dt;s.speed=v;s.timestamp=now
        if tick%300==0:
            print('DIAGNOSTIC',round(now),'completed',sum(s==WorkStatus.COMPLETE for s in e.dispatch.status.values()),'moving',sum(abs(s.speed)>.1 for s in states.values()),'diverted',sum(x['event']=='FRONT_VEHICLE_DIVERTED' for x in e.dispatch.events),flush=True)
            out.write_bytes(pickle.dumps(dict(executor=e,states=states,now=now)))
        finished=(e.dispatch.ready_to_finish() if args.complete_parking
                  else all(s==WorkStatus.COMPLETE for s in e.dispatch.status.values()))
        if finished:break
    out.write_bytes(pickle.dumps(dict(executor=e,states=states,now=now)))
    report=dict(scope='trusted checkpoint continuation only; not full-run or UE acceptance',
        start_s=start,end_s=now,completed_tasks=sum(s==WorkStatus.COMPLETE for s in e.dispatch.status.values()),
        expected_tasks=len(e.dispatch.status),parking_complete=e.dispatch.ready_to_finish(),failure=failure,
        collision_count=e.controller.collision_count,local_unauthorized_entry_count=e.controller.local_unauthorized_count,
        road_capacity_violation_steps=e.controller.capacity_violation_steps,parked_nodes=dict(e.dispatch.positions))
    (args.checkpoint.parent/'debug_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print('DIAGNOSTIC RESULT',json.dumps(report),flush=True)

if __name__=='__main__':main()
