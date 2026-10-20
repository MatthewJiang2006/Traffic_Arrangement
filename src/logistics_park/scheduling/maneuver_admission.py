"""Read-only prospective admission checks; no physical lease is released."""


def entered_special(plan, progress, start, end, *, forward_access=False):
    for i in range(start, min(len(plan.edges), plan.edge_index(end)+1)):
        edge=plan.model.edges[plan.edges[i]]
        if edge.get('reverse') or edge['kind']=='uturn' or forward_access and edge['kind']=='service_access':
            if plan.edge_starts[i]-.25 <= progress <= plan.edge_ends[i]+6.:
                return True
    return False


def prospective_cycle(local, vehicle, yielding, states, *, resolved_passage_start=None):
    from .traffic_transition import TrafficTransition,wait_cycle
    proposal=TrafficTransition('passage_admission',vehicle,
        waits=tuple((owner,vehicle) for owner in yielding),
        resolved=((vehicle,resolved_passage_start),) if resolved_passage_start is not None else ())
    return wait_cycle(local,states,proposal)
