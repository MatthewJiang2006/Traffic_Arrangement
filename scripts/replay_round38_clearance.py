#!/usr/bin/env python3
"""Replay round38's deadlocked clearance grant against the current policy.

Reconstructs the two trucks' exact plans from the saved diagnostics, rebuilds
their passage leases and calls the clearance admission used in the live run.
It proves whether the new duty check refuses the unfulfillable grant and
whether the committed reverse action keeps its precedence.
"""
import sys,math,json
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.runtime.transport_control import Execution,envelope
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic,sweeps_intersect
from logistics_park.safety.collision import overlaps,separating_gap

DIAG=ROOT/'runs/fleet16_reverse_precedence_round38/physical_control_diagnostics.jsonl'
SPEC=json.loads((ROOT/'config/runtime/industrialarea_ue_fleet16_flow_optimized.json').read_text())
NET=ROOT/SPEC['network'];MAP=ROOT/SPEC['map']

def frame():
    rows=[json.loads(l) for l in DIAG.open()]
    for row in reversed(rows):
        if any(r.get('vehicle')=='Car_0_1' for r in row.get('clearance_rights',[])):return row
    raise SystemExit('no frame with the round38 clearance right')

def plan_for(network,geometry,details):
    keys=['__test'.join(e.split('__test')[:-1]) or e for e in details['route_edges']]
    keys=list(dict.fromkeys(keys))
    origin=network.edges[keys[0]]['from_node']
    route=dict(edge_ids=keys)
    return geometry.plan(SimpleNamespace(origin=origin,destination=details['destination'],route=route))

def main():
    row=frame();print('frame t_s',row['t_s'])
    network=ServiceNetwork(NET,blocked_edges=SPEC['blocked_candidate_edges'])
    model=TransportMap(MAP);geometry=ServiceRouteGeometry(network,model)
    det=row['execution_details']
    plans={n:plan_for(network,geometry,det[n]) for n in ('Car_0_1','Car_1_1')}
    ex={n:Execution(plans[n],progress=det[n]['progress_m']) for n in plans}
    ex['Car_1_1'].checkpoint=det['Car_1_1']['checkpoint_index']
    ex['Car_0_1'].checkpoint=det['Car_0_1']['checkpoint_index']
    for n in plans:
        p=[round(v,3) for v in plans[n].point(ex[n].progress)[:2]]
        d=[round(v,3) for v in row['positions'][n][:2]]
        print(n,'plan progress pose',p,'measured',d,'reverse',plans[n].reverse(ex[n].progress))
    states={n:VehicleState(n,*row['positions'][n][:3],row['positions'][n][3]) for n in plans}
    pol=LocalTraffic(point_conflicts=True)
    for n in plans:
        lease=[p for p in row['local_permissions'] if p['vehicle_id']==n]
        for perm in lease:
            start=perm['start'];end=perm['end_m'];progress=ex[n].progress
            entry=max(0.,math.floor(progress*4)/4);limit=min(plans[n].length,end+11.344)
            count=max(1,math.ceil((limit-entry)/.25))
            stations=[(entry+(limit-entry)*i/count,envelope(VehicleState(n,*plans[n].point(entry+(limit-entry)*i/count),plans[n].heading(entry+(limit-entry)*i/count)))) for i in range(count+1)]
            pol.passages[n,start]=dict(plan=plans[n],start=start,end=end,sweep=[b for _,b in stations],stations=stations,created_at=0.)
            print(n,'lease start',start,'end',end,'age',perm['age_s'])
    owner='Car_1_1';grantee='Car_0_1'
    right=row['clearance_rights'][0];end=right['end_m']
    print('owner body vs grantee current body gap .35',
          round(separating_gap(envelope(states[grantee]),envelope(states[owner]),.35),4),
          'overlap .2',overlaps(envelope(states[grantee]),envelope(states[owner]),.2))
    print('unexecutable duty check:',not pol._clearance_executable(owner,grantee,ex[grantee],states,envelope,end))
    pol._protect_maneuver_space(states,ex,envelope)
    print('clearance_rights after prepare:',list(pol.clearance_rights))
    print('clearance_pauses after prepare:',{k:sorted(v) for k,v in pol.clearance_pauses.items()})
    print('maneuver_holds after prepare:',{k:{kk:vv for kk,vv in v.items()} for k,v in pol.maneuver_holds.items()})
    for e in pol.events:print('event',e)
    result=pol.regulate(states,ex,{grantee:(8.,0.),owner:(-1.,0.)},{},row['t_s'],envelope)
    print('commands',{n:round(v[0],3) for n,v in result.items()})
    return 0

if __name__=='__main__':raise SystemExit(main())
