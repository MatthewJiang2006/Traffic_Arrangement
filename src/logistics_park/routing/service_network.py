"""Directed service-node routing, preserving lane and access alternatives."""
import json
from collections import defaultdict, deque
from heapq import heappop, heappush
from pathlib import Path
from logistics_park.common.messages import finite


class ServiceNetwork:
    def __init__(self, path, *, blocked_edges=frozenset()):
        self.raw=json.loads(Path(path).read_text())
        self.nodes={n['node_id']:n for n in self.raw['nodes']}
        self.edges={e['edge_id']:e for e in self.raw['edges']}
        self.blocked_edges=frozenset(blocked_edges)
        if self.blocked_edges-set(self.edges):raise ValueError('Unknown blocked service edge')
        self.services=self.raw['service_nodes']
        self.outgoing=defaultdict(list);self.incoming=defaultdict(list)
        self.successors=defaultdict(list);self.predecessors=defaultdict(list)
        for key,e in self.edges.items():
            self.outgoing[e['from_node']].append(key);self.incoming[e['to_node']].append(key)
        for movement in self.raw['movements']:
            a,b=movement['incoming_edge'],movement['outgoing_edge']
            self.successors[a].append(b);self.predecessors[b].append(a)

    def __getstate__(self):
        # Runtime LRU wrappers are not data; rebuild them after a diagnostic load.
        return {k:v for k,v in self.__dict__.items() if k!='route'}

    def route(self, origin, destination, *, first_edge=None, last_edge=None,incoming_edge=None,
              occupied_nodes=frozenset(), allow_pending_access=False, edge_costs=None, transit_warehouses=None):
        if (origin not in self.nodes or destination not in self.services or origin==destination
                or (origin not in self.services and incoming_edge is None)):
            raise ValueError('distinct known service nodes required')
        if incoming_edge is not None and (incoming_edge not in self.edges or self.edges[incoming_edge]['to_node']!=origin):
            raise ValueError('Incoming edge does not reach the route origin')
        if set(occupied_nodes)-set(self.services):raise ValueError('unknown occupied service nodes')
        delays=edge_costs or {}
        if set(delays)-set(self.edges):raise ValueError('Unknown traffic-cost edge')
        finite(*delays.values())
        if any(v<0 for v in delays.values()):raise ValueError('Negative traffic delay')
        def weight(key):return self.edges[key]['estimated_travel_time_s']+delays.get(key,0.)
        occupied_resources={r for n in set(occupied_nodes)-{origin}
                            for r in self.services[n].get('occupation_resources',[])}
        if first_edge is not None and first_edge not in self.outgoing[origin]:raise ValueError('invalid departure')
        if last_edge is not None and last_edge not in self.incoming[destination]:raise ValueError('invalid arrival')
        best={};queue=[]
        # Waiting bays are destinations, not road shortcuts. Tandem bays may
        # explicitly require traversing a neighbouring bay on entry/departure.
        waiting_dependencies=set(self.services.get(origin,{}).get('departure_requires_empty',[]))
        waiting_dependencies.update(self.services[destination].get('departure_requires_empty',[]))
        def allowed(key):
            e=self.edges[key]
            site=self.services.get(e['to_node'])
            if (site and site['kind']=='waiting' and e['to_node'] not in (origin,destination)
                    and e['to_node'] not in waiting_dependencies):return False
            if (transit_warehouses is not None and site and site['kind'] in ('loading','unloading')
                    and e['to_node'] not in (origin,destination) and site.get('warehouse') not in transit_warehouses):return False
            return (key not in self.blocked_edges and not set(e.get('must_be_empty_nodes',[])) & set(occupied_nodes)
                    and not set(e.get('required_resources',[])) & occupied_resources
                    and e['to_node'] not in occupied_nodes
                    and (allow_pending_access or not (e.get('attachment_validation_required',False) or e.get('access_validation_required',False))))
        for key in (self.successors[incoming_edge] if incoming_edge is not None else self.outgoing[origin]):
            if (first_edge is None or key==first_edge) and allowed(key):
                cost=weight(key);best[key]=(cost,(key,));heappush(queue,(cost,(key,),key))
        while queue:
            cost,path,key=heappop(queue)
            if best.get(key)!=(cost,path):continue
            if self.edges[key]['to_node']==destination and (last_edge is None or last_edge==key):
                return dict(edge_ids=list(path),distance_m=sum(self.edges[k]['length_m'] for k in path),
                    estimated_travel_time_s=cost,
                    freeflow_travel_time_s=sum(self.edges[k]['estimated_travel_time_s'] for k in path),
                    congestion_delay_s=sum(delays.get(k,0.) for k in path),
                    attachment_validation_required=any(self.edges[k].get('attachment_validation_required',False) for k in path),
                    access_validation_required=any(self.edges[k].get('access_validation_required',False) for k in path),
                    initial_direction=-1 if self.edges[path[0]].get('reverse') else 1,
                    gear_change_nodes=([origin] if self.edges[path[0]].get('reverse') else [])+[self.edges[a]['to_node'] for a,b in zip(path,path[1:])
                                       if bool(self.edges[a].get('reverse'))!=bool(self.edges[b].get('reverse'))],
                    required_resources=sorted({r for k in path for r in self.edges[k].get('required_resources',[])}),
                    must_be_empty_nodes=sorted({n for k in path for n in self.edges[k].get('must_be_empty_nodes',[])} |
                        {self.edges[k]['to_node'] for k in path if self.edges[k]['to_node'] in waiting_dependencies
                         and self.edges[k]['to_node'] not in (origin,destination)}))
            # Positive costs select finite routes; the full graph retains detours.
            for nxt in self.successors[key]:
                if not allowed(nxt):continue
                candidate=(cost+weight(nxt),path+(nxt,))
                if nxt not in best or candidate<best[nxt]:
                    best[nxt]=candidate;heappush(queue,(*candidate,nxt))
        return None

    def reachable_edges(self, seeds, reverse=False):
        adjacency=self.predecessors if reverse else self.successors
        seen=set(seeds);queue=deque(seeds)
        while queue:
            for key in adjacency[queue.popleft()]:
                if key not in seen:seen.add(key);queue.append(key)
        return seen

    def corridor(self, origin, destination):
        """All edges on some legal directed walk, including possible detours."""
        return sorted(self.reachable_edges(self.outgoing[origin]) &
                      self.reachable_edges(self.incoming[destination],reverse=True))

    def queue_options(self, origin, destination, routes=None, *, allow_pending_access=False,
                      occupied_nodes=frozenset()):
        if occupied_nodes:routes=None  # A static cache cannot override live occupancy.
        result=[]
        for q,node in self.services.items():
            if node['kind']!='waiting' or q in (origin,destination):continue
            before=routes[(origin,q)] if routes is not None else self.route(origin,q,allow_pending_access=allow_pending_access,occupied_nodes=occupied_nodes)
            after=routes[(q,destination)] if routes is not None else self.route(q,destination,allow_pending_access=allow_pending_access,occupied_nodes=occupied_nodes)
            if not before or not after:continue
            baseline=routes[(origin,destination)] if routes is not None else self.route(origin,destination,allow_pending_access=allow_pending_access,occupied_nodes=occupied_nodes)
            traversed={self.edges[k]['to_node'] for k in before['edge_ids']}
            requires_berth_clear=destination in traversed or destination in before['must_be_empty_nodes']
            conflicts=sorted(set(before['required_resources']) & set(self.services[destination].get('occupation_resources',[])))
            requires_destination_clear=requires_berth_clear or bool(conflicts)
            on_corridor=q in {self.edges[k]['to_node'] for k in self.corridor(origin,destination)}
            result.append(dict(waiting_node=q,capacity=node['capacity'],
                approach=before,onward=after,
                extra_distance_m=before['distance_m']+after['distance_m']-baseline['distance_m'] if baseline else None,
                assigned_to_destination=bool(node.get('assigned_warehouses',[]) and self.services[destination].get('warehouse') in node['assigned_warehouses']),
                on_legal_corridor=on_corridor,requires_destination_clear=requires_destination_clear,
                requires_destination_berth_clear=requires_berth_clear,destination_resource_conflicts=conflicts,
                usable_while_destination_occupied=not requires_destination_clear,
                departure_requires_empty=node.get('departure_requires_empty',[]),
                required_resources=sorted(set(before['required_resources']+after['required_resources'])),
                attachment_validation_required=before['attachment_validation_required'] or after['attachment_validation_required'],
                access_validation_required=before['access_validation_required'] or after['access_validation_required'],
                operationally_enabled=False))
        return sorted(result,key=lambda r:(r['requires_destination_clear'],r['attachment_validation_required'] or r['access_validation_required'],r['extra_distance_m'],r['waiting_node']))
