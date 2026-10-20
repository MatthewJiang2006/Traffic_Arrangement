"""Reorder stopped cycles by withdrawing only unentered, brake-safe promises."""
import math
from logistics_park.scheduling.wait_dependencies import wait_snapshot
from .transport_control import envelope


def recover_wait_cycles(executor,states,commands,now):
    if not executor.dispatch.efficiency_policy:return
    if now-getattr(executor,'cycle_checked_at',-math.inf)<1.:return
    executor.cycle_checked_at=now
    snapshot=wait_snapshot(executor,states)
    local=executor.controller.local_traffic
    if not local:return
    previous=getattr(executor,'cycle_observations',{});current={}
    for members in snapshot['cycles']:
        key=tuple(members)
        if any(abs(states[n].speed)>=.15 or abs(commands.get(n,(0,0))[0])>=.15 for n in members):continue
        old=previous.get(key)
        if old is None or any(math.dist(old['xy'][n],(states[n].x,states[n].y))>=.25 for n in members):
            old=dict(since=now,xy={n:(states[n].x,states[n].y) for n in members},checked=False)
        current[key]=old
        if now-old['since']<10. or old['checked']:continue
        old['checked']=True;withdrawn=[]
        for passage,lease in list(local.passages.items()):
            name,start=passage
            if name not in members:continue
            ex=executor.controller.executions[name]
            if not local.release_passages((passage,),states,envelope,reason='cycle_revocation',
                                          executions=executor.controller.executions):continue
            ex.local_passages.discard(start);withdrawn.append(dict(vehicle=name,start=start))
        executor.dispatch._event('WAIT_CYCLE_REEVALUATED',vehicles=members,
            withdrawn_unentered=withdrawn,physical_leases_revoked=False)
    executor.cycle_observations=current
