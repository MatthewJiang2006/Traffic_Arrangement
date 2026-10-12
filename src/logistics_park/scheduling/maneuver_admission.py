"""Read-only prospective admission checks; no physical lease is released."""
from .wait_dependencies import strongly_connected_cycles


def entered_special(plan, progress, start, end, *, forward_access=False):
    for i in range(start, min(len(plan.edges), plan.edge_index(end)+1)):
        edge=plan.model.edges[plan.edges[i]]
        if edge.get('reverse') or edge['kind']=='uturn' or forward_access and edge['kind']=='service_access':
            if plan.edge_starts[i]-.25 <= progress <= plan.edge_ends[i]+6.:
                return True
    return False


def prospective_cycle(local, vehicle, yielding, states, *, resolved_passage_start=None):
    rows=[]
    for row in getattr(local,'admission_dependencies',()):
        a,b=row['vehicle'],row['blocked_by']
        # claim_passage has just rechecked body/braking/sweep clearance. The
        # previous denial for this same request is replaced by the proposal;
        # carrying it forward would invent an applicant -> yielding owner ->
        # applicant cycle and permanently reject an otherwise clear crossing.
        if (resolved_passage_start is not None and a==vehicle
                and row.get('resource')=='passage:'+str(resolved_passage_start)):
            continue
        if a in states and b in states and abs(states[a].speed)<.15:
            rows.append(dict(vehicle=a,blocked_by=b))
    for owner,grants in local.paused_by.items():
        rows.extend(dict(vehicle=owner,blocked_by=g[0]) for g in grants)
    for owner,grantees in local.clearance_pauses.items():
        rows.extend(dict(vehicle=owner,blocked_by=g) for g in grantees)
    rows.extend(dict(vehicle=owner,blocked_by=vehicle) for owner in yielding)
    return next((cycle for cycle in strongly_connected_cycles(rows)
                 if vehicle in cycle and set(cycle).intersection(yielding)),None)
