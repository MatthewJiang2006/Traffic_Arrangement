#!/usr/bin/env python3
"""Fast discrete-event validation of task decisions on the real service graph.

Travel finishes after route ETA; no vehicle dynamics or collision proof is
claimed. UE/kinematic execution must supply the same departure/arrival events.
"""
import heapq
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry,FleetPhase,GatePermit,GateDirection
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork,WorkStatus


def run(name,positions,work,dynamic=False):
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json')
    fleet=FleetRegistry();d=TaskDispatcher(network,fleet)
    for vehicle,node in positions.items():
        v=fleet.adopt_present_vehicle(vehicle,0,fully_inside=True,resources=[])
        d.place_vehicle(vehicle,v.visit_id,node,0,stopped=True)
    for task in work:d.submit(task)
    events=[];serial=0;now=0.;failure=None;peak=0;joined=False;exit_requested=False
    for iteration in range(10000):
        while events and events[0][0]<=now+1e-8:
            _,_,kind,move=heapq.heappop(events)
            if kind=='departure':d.departed(move.command_id,move.vehicle_id,move.visit_id,now,tail_clear=True)
            else:d.arrived(move.command_id,move.vehicle_id,move.visit_id,now,stopped=True)
        if dynamic and not joined and now>=20 and 'Q12' not in d.holders:
            v=fleet.register_vehicle('new-truck',now)
            fleet.request_entry(v.vehicle_id,v.visit_id,'logical-entry',now)
            fleet.authorize_entry(GatePermit('entry',v.vehicle_id,v.visit_id,'logical-entry',GateDirection.ENTRY,now,now+1),now)
            fleet.confirm_entry(v.vehicle_id,v.visit_id,now,fully_inside=True)
            d.place_vehicle(v.vehicle_id,v.visit_id,'Q12',now,stopped=True);joined=True
        if dynamic and not exit_requested and fleet.vehicles['old-truck'].phase==FleetPhase.ON_TASK:
            v=fleet.vehicles['old-truck'];fleet.request_exit(v.vehicle_id,v.visit_id,'logical-exit',now);exit_requested=True
        commands=d.tick(now)
        for move in commands:
            for kind,time in (('departure',now+.1),('arrival',now+max(.2,move.route['estimated_travel_time_s']))):
                serial+=1;heapq.heappush(events,(time,serial,kind,move))
        if dynamic and fleet.vehicles['old-truck'].phase==FleetPhase.EXIT_REQUESTED:
            v=fleet.vehicles['old-truck']
            fleet.authorize_exit(GatePermit('exit',v.vehicle_id,v.visit_id,'logical-exit',GateDirection.EXIT,now,now+1),now)
            d.release_for_exit(v.vehicle_id,v.visit_id,now,tail_clear=True)
            fleet.set_owned_resources(v.vehicle_id,v.visit_id,[],now)
            fleet.confirm_exit(v.vehicle_id,v.visit_id,now,fully_outside=True)
        peak=max(peak,len(d.moves))
        if all(s==WorkStatus.COMPLETE for s in d.status.values()):break
        upcoming=[t for t,*_ in events]
        upcoming += [a.service_until for a in d.assignments.values() if a.service_until is not None and a.service_until>now]
        upcoming += [t.release_time for t in d.tasks.values() if d.status[t.task_id]==WorkStatus.PENDING and t.release_time>now]
        if dynamic and not joined and now<20:upcoming.append(20.)
        if not upcoming:
            failure='No future progress event; possible deadlock';break
        now=min(upcoming)
    else:failure='Event budget exhausted'
    completed=sum(s==WorkStatus.COMPLETE for s in d.status.values())
    outcome=dict(status='PASS' if completed==len(work) and not failure else 'FAIL',scenario=name,
                 expected_tasks=len(work),completed_tasks=completed,elapsed_dispatch_s=now,
                 maximum_concurrent_moves=peak,waiting_moves=sum(e['event']=='MOVE_REQUESTED' and e['waiting'] for e in d.events),
                 new_vehicle_joined=joined,old_vehicle_departed=dynamic and fleet.vehicles['old-truck'].phase==FleetPhase.DEPARTED,
                 ue_physics_validated=False,vehicle_dynamics_validated=False,
                 scope='Discrete-event task/lease contract only; travel uses static route ETA',failure=failure,
                 tasks={k:v.value for k,v in d.status.items()},events=d.events,fleet_events=fleet.events,
                 remaining_positions=d.positions,remaining_assignments={v:dict(task=a.work.task_id,state=a.state,target=d._target(a)) for v,a in d.assignments.items()})
    print(name,outcome['status'],completed,'/',len(work),'queued',outcome['waiting_moves'],'failure',failure,flush=True)
    return outcome


def main():
    work=[TransportWork('transport-'+str(i),f'W{i%6+1}-loading',f'W{(i+1)%6+1}-unloading') for i in range(12)]
    saturated=[TransportWork('queue-'+str(i),'W5-loading','W6-unloading') for i in range(4)]
    results=[run('four_vehicles_six_warehouses',{'car-a':'Q3','car-b':'Q6','car-c':'Q9','car-d':'Q11'},work),
             run('busy_destination_waiting',{'car-a':'Q2','car-b':'Q9','car-c':'Q11'},saturated),
             run('dynamic_entry_exit',{'old-truck':'Q9','car-b':'Q3','car-c':'Q6'},work,dynamic=True)]
    output=ROOT/'runs/task_dispatch';output.mkdir(parents=True,exist_ok=True)
    status='PASS' if all(r['status']=='PASS' for r in results) else 'FAIL'
    (output/'acceptance.json').write_text(json.dumps(dict(status=status,scenarios=results,ue_physics_validated=False),indent=2)+'\n')
    return 0 if status=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
