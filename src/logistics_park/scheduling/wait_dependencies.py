"""Explain physical and logical waiting without revoking any live lease."""
import math

def strongly_connected_cycles(edges):
    adjacency={}
    for e in edges:adjacency.setdefault(e['vehicle'],set()).add(e['blocked_by'])
    nodes=set(adjacency)|{v for vs in adjacency.values() for v in vs}
    index=0;indices={};low={};stack=[];active=set();cycles=[]
    def visit(v):
        nonlocal index
        indices[v]=low[v]=index;index+=1;stack.append(v);active.add(v)
        for w in sorted(adjacency.get(v,())):
            if w not in indices:visit(w);low[v]=min(low[v],low[w])
            elif w in active:low[v]=min(low[v],indices[w])
        if low[v]==indices[v]:
            component=[]
            while True:
                w=stack.pop();active.remove(w);component.append(w)
                if w==v:break
            if len(component)>1 or v in adjacency.get(v,()):cycles.append(sorted(component))
    for v in sorted(nodes):
        if v not in indices:visit(v)
    return sorted(cycles)

def wait_snapshot(executor,states):
    """Snapshot dependency evidence, not proof that every cycle is permanent."""
    d=executor.dispatch;c=executor.controller;rows=[];active_rows=[]
    # Controllers record guards throughout their prediction window. A guard
    # ahead of a stopped truck is not its current wait: another permission may
    # be stopping it first. Only reached holding lines form wait-for edges.
    def at_hold(ex,station):return ex.progress>=station-.5
    def add(v,other,reason,resource,*,active=True):
        if v!=other:
            row=dict(vehicle=v,blocked_by=other,reason=reason,resource=resource)
            rows.append(row)
            if active:active_rows.append(row)
    for vehicle,a in d.assignments.items():
        if a.state not in ('READY','WAITING') or vehicle not in d.positions:continue
        target=d._target(a)
        head=getattr(d,'approach_intents',{}).get(target)
        if head not in (None,vehicle):add(vehicle,head,'destination_queue_intent',target)
        for other in d._blockers(target,vehicle):add(vehicle,other,'destination_or_access',target)
        origin=d.positions[vehicle]
        if origin==target:continue
        direct=d.network.route(origin,target)
        if direct:
            required=set(direct.get('required_resources',[]));empty=set(direct.get('must_be_empty_nodes',[]))
            for node,holder in d.holders.items():
                if node in empty or required.intersection(d.network.services[node].get('occupation_resources',[])):
                    add(vehicle,holder,'route_clearance',node)
    for vehicle,state in states.items():
        ex=c.executions[vehicle]
        if ex.finished:continue
        if abs(state.speed)<.1:
            for denial in getattr(c,'control_denials',{}).get(vehicle,[]):
                if denial.get('blocked_by'):
                    add(vehicle,denial['blocked_by'],denial['reason'],'motion_guard',
                        active=denial.get('distance_m',0.)<=.5)
            move=d.moves.get(vehicle)
            if move and vehicle in c.destination_holds:
                for other in d._blockers(move.destination,vehicle):
                    add(vehicle,other,'approach_berth',move.destination,
                        active=at_hold(ex,c.destination_holds[vehicle]))
                head=getattr(d,'approach_intents',{}).get(move.destination)
                hold=c.destination_holds[vehicle]
                if ex.progress>=hold-.5 and head not in (None,vehicle):
                    add(vehicle,head,'destination_queue_intent',move.destination)
            if c.local_traffic:
                for grant in getattr(c.local_traffic,'paused_by',{}).get(vehicle,set()):add(vehicle,grant[0],'paused_for_clear_passage','passage:'+str(grant[1]))
                for denial in getattr(c.local_traffic,'passage_denials',{}).get(vehicle,[]):
                    add(vehicle,denial['blocked_by'],denial['reason'],'passage:'+str(denial['start']),
                        active=at_hold(ex,ex.plan.edge_starts[denial['start']]-7.8))
                denial=c.local_traffic.last_denials.get(vehicle,{})
                if denial.get('blocked_by'):add(vehicle,denial['blocked_by'],denial['reason'],'local_trajectory')
                if denial.get('reason')=='capacity':
                    alias=denial['resource']
                    for other in c.capacity.occupants[alias]|c.capacity.committed[alias]:add(vehicle,other,'downstream_capacity',alias)
        for i,v in enumerate(ex.plan.shared_visits):
            if i in ex.shared_cleared or i in ex.shared_entered or v.start-ex.progress>25:continue
            holder=c.shared_scheduler.owners.get(v.channel_id) if c.shared_scheduler else None
            if holder is not None:add(vehicle,holder,'shared_region',v.channel_id)
        for other,other_state in states.items():
            if other==vehicle:continue
            error,leader=ex.plan.project((other_state.x,other_state.y),ex.progress,behind=0,ahead=35)
            angle=abs((other_state.heading-ex.plan.heading(leader)+math.pi)%(2*math.pi)-math.pi)
            if error<1.4 and angle<math.pi/3 and ex.progress+.1<leader<ex.progress+35:
                add(vehicle,other,'physical_leader','route')
    def unique(rows):
        return sorted({tuple(sorted(row.items())):row for row in rows}.values(),key=lambda row:tuple(row.values()))
    rows=unique(rows);active_rows=unique(active_rows)
    # Keep predictive evidence for proactive admission, which must avoid
    # future blocking chains before entering a maneuver. The stagnation watch
    # instead needs the dependencies actually holding the trucks now.
    return dict(dependencies=rows,active_dependencies=active_rows,
        cycles=strongly_connected_cycles(active_rows),physical_leases_revoked=False)
