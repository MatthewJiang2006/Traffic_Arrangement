"""Replace only the uncommitted future of a stopped truck with legal graph edges."""
import math
from dataclasses import replace


def try_conflict_detours(executor,states,now):
    c=executor.controller;d=executor.dispatch;local=c.local_traffic
    if not local or not c.model.raw.get('legal_conflict_detours',False):return
    if not hasattr(executor,'detour_since'):executor.detour_since={};executor.detour_attempt={}
    for name,state in states.items():
        if abs(state.speed)>.08:executor.detour_since.pop(name,None);continue
        since=executor.detour_since.setdefault(name,now)
        if now-since<2 or now-executor.detour_attempt.get(name,-100)<4:continue
        ex=c.executions[name];move=executor.moves.get(name)
        reasons=local.passage_denials.get(name,[])
        denial=local.last_denials.get(name,{})
        blocker=denial.get('blocked_by') or next((r['blocked_by'] for r in reasons if r.get('blocked_by') in states),None)
        if (not move or ex.finished or blocker not in states or any(who==name for who,_ in local.passages)
                or c.model.edges[ex.plan.edges[ex.plan.edge_index(ex.progress)]]['kind']!='road'):continue
        executor.detour_attempt[name]=now
        network=d.network;costs=d.traffic_routing.costs_for(name,d.moves) if d.traffic_routing else {}
        other=c.executions[blocker]
        aliases=set()
        for i in range(other.plan.edge_index(other.progress),len(other.plan.edges)):
            if other.plan.edge_starts[i]-other.progress>25:break
            aliases.add(other.plan.model.aliases[other.plan.edges[i]])
        for key,edge in network.edges.items():
            if edge['capacity_alias'] in aliases:costs[key]=costs.get(key,0)+30
        station=0.;prefix=[]
        for key in move.route['edge_ids']:
            prefix.append(key);edge=network.edges[key];station+=edge['length_m']
            if station<ex.progress+6:continue
            if station>ex.progress+40:break
            if edge['kind']!='road':continue
            suffix=network.route(edge['to_node'],move.destination,incoming_edge=key,
                occupied_nodes=d._occupied(name),edge_costs=costs,
                transit_warehouses=frozenset({network.services[move.destination]['warehouse']})
                    if network.services[move.destination]['kind'] in ('loading','unloading')
                    else frozenset())
            if not suffix:continue
            if move.waiting and not d._parking_approach_clear(suffix,name):continue
            keys=prefix+suffix['edge_ids']
            if keys==move.route['edge_ids']:continue
            policy=d.efficiency_policy
            if policy and sum(network.edges[k]['length_m'] for k in keys)>move.route['distance_m']+policy.waiting_extra_m:continue
            route=dict(suffix,edge_ids=keys,distance_m=sum(network.edges[k]['length_m'] for k in keys),
                estimated_travel_time_s=sum(network.edges[k]['estimated_travel_time_s']+costs.get(k,0) for k in keys),
                required_resources=sorted({r for k in keys for r in network.edges[k].get('required_resources',[])}),
                must_be_empty_nodes=sorted({n for k in keys for n in network.edges[k].get('must_be_empty_nodes',[])}))
            candidate=replace(move,route=route)
            try:
                plan=executor.geometry.plan(candidate)
                # A detour must actually leave the blocked future corridor.
                old_future=move.route['edge_ids'][len(prefix):]
                old_cost=sum(costs.get(k,0) for k in old_future)
                new_cost=sum(costs.get(k,0) for k in suffix['edge_ids'])
                if new_cost>=old_cost:continue
                c.reroute_stopped(name,plan,now,state=state)
            except ValueError:continue
            executor.moves[name]=candidate;d.moves[name]=candidate
            d._event('LEGAL_CONFLICT_DETOUR_SELECTED',vehicle_id=name,blocked_by=blocker,
                destination=move.destination,edge_ids=keys,retained_prefix_m=ex.progress+6)
            break
