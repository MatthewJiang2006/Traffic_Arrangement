"""Bounded rolling task planning with a shared, advisory berth calendar."""
from dataclasses import dataclass,replace
import math
import time
from .optimization import bottleneck_assignment
from .resource_calendar import ResourceCalendar


@dataclass(frozen=True)
class Candidate:
    vehicle: str
    work: object
    pickup_s: float
    loaded_s: float
    empty_s: float
    parking_s: float
    approach_s: float
    approach_spans: tuple=()
    loaded_spans: tuple=()


def berth_resources(dispatch,node):
    return ('berth:'+node,)+tuple(dispatch.network.services[node].get('occupation_resources',[]))


class RollingScheduler:
    def __init__(self,budget_s=.02):
        if not math.isfinite(budget_s) or budget_s<=0:raise ValueError('Invalid planning budget')
        self.budget_s=budget_s;self.last_plan=None
        self.parking_routes={};self.span_cache={};self.route_skeletons={}

    def spans(self,dispatch,route,duration):
        edges=getattr(dispatch.network,'edges',{})
        if not edges:return ()
        keys=tuple(route['edge_ids']);tag=(keys,round(duration,3))
        if tag in self.span_cache:return self.span_cache[tag]
        nominal=sum(edges[k]['estimated_travel_time_s'] for k in keys)
        scale=duration/max(.001,nominal);rows=[];at=0.
        for key in keys:
            edge=edges[key];dt=edge['estimated_travel_time_s']*scale
            resources=tuple(sorted(set(edge.get('required_resources',[]))|{'road:'+edge['capacity_alias']}))
            if rows and rows[-1][1]==resources:
                old=rows.pop();rows.append((old[0],resources,old[2]+dt))
            else:rows.append((at,resources,dt))
            at+=dt
        # A prediction retains space until the rear clears each action.
        result=tuple((offset,resources,dt+2.) for offset,resources,dt in rows)
        self.span_cache[tag]=result
        if len(self.span_cache)>2048:self.span_cache={tag:result}
        return result

    def prepare(self,dispatch):
        """Compile immutable parking skeletons before measured driving starts."""
        targets=[n for n,s in dispatch.network.services.items() if s['kind'] in ('loading','unloading')]
        for origin in dispatch.network.services:
            for target in targets:
                if origin==target:continue
                route=dispatch.network.route(origin,target,occupied_nodes=frozenset(),
                    **({'transit_warehouses':dispatch._traffic_warehouses(origin,target)} if dispatch.traffic_routing else {}))
                if route:self.route_skeletons[origin,target]=route
        for origin,site in dispatch.network.services.items():
            if site['kind']!='unloading':continue
            rows=[]
            for q,target in dispatch.network.services.items():
                if target['kind']!='waiting':continue
                if dispatch.efficiency_policy and not dispatch.efficiency_policy.parking_enabled(q):continue
                route=dispatch.network.route(origin,q,occupied_nodes=frozenset(),
                    **({'transit_warehouses':dispatch._traffic_warehouses(origin,q)} if dispatch.traffic_routing else {}))
                if route:rows.append((q,route))
            self.parking_routes[origin]=rows

    def candidate_route(self,dispatch,origin,target,vehicle):
        if origin==target:return dict(edge_ids=[],distance_m=0.,estimated_travel_time_s=0.)
        route=self.route_skeletons.get((origin,target))
        if route is None:return None
        occupied=dispatch._occupied(vehicle)-{target}
        if occupied.intersection(route.get('must_be_empty_nodes',[])):return None
        delay=0.
        if dispatch.traffic_routing:
            costs=dispatch.traffic_routing.costs_for(vehicle,dispatch.moves)
            delay=sum(costs.get(key,0.) for key in route['edge_ids'])
        if dispatch.traffic_forecast:
            delay=max(delay,dispatch.traffic_forecast.route_delay(dispatch.network,route,vehicle,dispatch.now))
        nominal=route.get('freeflow_travel_time_s',route['estimated_travel_time_s'])
        return {**route,'congestion_delay_s':delay,'estimated_travel_time_s':nominal+delay}

    def calendar(self,dispatch):
        capacities={}
        if dispatch.traffic_routing:capacities={'road:'+key:int(value) for key,value in dispatch.traffic_routing.capacities.items()}
        calendar=ResourceCalendar(capacities)
        forecast=dispatch.traffic_forecast
        if forecast:
            for interval in forecast.intervals:
                calendar.reserve(['road:'+interval.alias],interval.start,interval.end-interval.start,interval.vehicle)
        for name,a in dispatch.assignments.items():
            target=dispatch._target(a)
            if a.state=='SERVICING':
                calendar.reserve(berth_resources(dispatch,target),dispatch.now,
                                 max(0.,a.service_until-dispatch.now)+20.,name)
                continue
            eta=dispatch._admission_priority(a,dispatch.admission_priorities)[0]
            if math.isfinite(eta):
                calendar.reserve(berth_resources(dispatch,target),dispatch.now+max(0.,eta),
                                 dispatch.service_s+20.,name)
        return calendar

    def ready_departures(self,dispatch):
        """Fresh source waiting suggestions for loaded/ready real assignments."""
        calendar=self.calendar(dispatch);result={};deadline=time.monotonic()+self.budget_s
        for name,a in sorted(dispatch.assignments.items(),key=lambda item:(item[1].queued_at or dispatch.now,item[0])):
            if a.state not in ('READY','WAITING') or name not in dispatch.positions or name in dispatch.moves:continue
            origin=dispatch.positions[name];target=dispatch._target(a)
            if (origin==target or dispatch.network.services[origin]['kind']=='staging'
                    or dispatch._source_waiters(origin,name)):continue
            if time.monotonic()>deadline:break
            route=dispatch._route(origin,target,name,future=True)
            if not route:continue
            duration=dispatch._score_route(origin,target,route)
            departure=calendar.trajectory(self.spans(dispatch,route,duration),dispatch.now,name,
                                          latest_start=dispatch.now+600.)
            if departure is not None and departure>dispatch.now+1.:
                result[name]=(a.work.task_id,a.operation,origin,departure)
        return result

    @staticmethod
    def evaluate(dispatch,candidates,calendar):
        calendar=calendar.copy();finishes=[];waits=[];empty=0.;details=[]
        for c in candidates:
            start=dispatch.now
            while True:
                trial=calendar.copy()
                departure=trial.trajectory(c.approach_spans,start,c.vehicle,latest_start=dispatch.now+600.)
                if departure is None:return math.inf,[]
                arrival=departure+c.approach_s
                loading=trial.earliest(berth_resources(dispatch,c.work.loading_node),
                                      max(arrival,dispatch.now+c.pickup_s),dispatch.service_s+20.,ignore=c.vehicle)
                if loading<=arrival+1e-6:break
                start=loading-c.approach_s
            end=trial.reserve(berth_resources(dispatch,c.work.loading_node),loading,dispatch.service_s+20.,c.vehicle)
            base=trial.copy();start=end
            while True:
                trial=base.copy()
                loaded_departure=trial.trajectory(c.loaded_spans,start,c.vehicle,latest_start=end+600.)
                if loaded_departure is None:return math.inf,[]
                arrival=loaded_departure+c.loaded_s
                unloading=trial.earliest(berth_resources(dispatch,c.work.unloading_node),arrival,
                                        dispatch.service_s+20.,ignore=c.vehicle)
                if unloading<=arrival+1e-6:break
                start=unloading-c.loaded_s
            finish=trial.reserve(berth_resources(dispatch,c.work.unloading_node),unloading,dispatch.service_s+20.,c.vehicle)
            # A delayed loaded departure keeps its source service space occupied.
            if loaded_departure>end:trial.reserve(berth_resources(dispatch,c.work.loading_node),end,loaded_departure-end,c.vehicle)
            calendar=trial
            finishes.append(finish-dispatch.now+c.parking_s)
            waits.append(departure-dispatch.now+loading-departure-c.pickup_s+unloading-end-c.loaded_s)
            empty+=c.empty_s
            details.append(dict(vehicle=c.vehicle,task=c.work.task_id,loading_at=loading,
                                unloading_at=unloading,finish_at=finish,parking_s=c.parking_s,
                                loaded_departure_at=loaded_departure,not_before=departure))
        if not finishes:return 0.,details
        return max(finishes)+.3*sum(finishes)/len(finishes)+.2*sum(waits)+.15*empty,details

    def plan(self,dispatch,pending):
        started=time.monotonic();deadline=started+self.budget_s
        vehicles=[v for v in sorted(dispatch.fleet.available_vehicle_ids)
                  if v in dispatch.positions and v not in dispatch.moves]
        if not vehicles or not pending:
            self.last_plan=dict(status='NO_AVAILABLE_CANDIDATES',snapshot_time=dispatch.now,
                                elapsed_s=time.monotonic()-started,physical_leases_changed=False)
            return []
        # Explicit business priorities are never traded for shorter distances.
        priority=max(w.priority for w in pending)
        work=[w for w in pending if w.priority==priority][:64]
        matrix=[];lookup={};calendar=self.calendar(dispatch)
        parking_cache={};pair_cache={}
        for i,name in enumerate(vehicles):
            row=[];origin=dispatch.positions[name]
            for j,w in enumerate(work):
                if time.monotonic()>deadline:
                    self.last_plan=dict(status='BUDGET_EXCEEDED',elapsed_s=time.monotonic()-started,
                                        candidate_pairs_built=len(lookup),matrix_rows_built=len(matrix))
                    return None
                prototype=pair_cache.get((name,w.loading_node,w.unloading_node))
                if prototype is not None:
                    c,cost=prototype;lookup[i,j]=replace(c,work=w)
                    row.append(max(0.,cost-min(10.,max(0.,dispatch.now-w.release_time)/60.)))
                    continue
                counts=sum(a.operation=='LOADING' and a.work.loading_node==w.loading_node for a in dispatch.assignments.values())
                if dispatch.scheduling.max_loading_inflight and counts>=dispatch.scheduling.max_loading_inflight:
                    row.append(math.inf);continue
                if not self.route_skeletons:self.prepare(dispatch)
                route=self.candidate_route(dispatch,origin,w.loading_node,name)
                onward=self.candidate_route(dispatch,w.loading_node,w.unloading_node,name)
                if not route or not onward:row.append(math.inf);continue
                pickup=dispatch._pickup_eta(w,name,route)
                loaded=dispatch._score_route(w.loading_node,w.unloading_node,onward)
                if w.unloading_node not in parking_cache:
                    options=[]
                    rows=self.parking_routes.get(w.unloading_node)
                    if rows is None:
                        # Small standalone dispatchers can prepare lazily; the
                        # native executor prepares outside its control loop.
                        self.prepare(dispatch);rows=self.parking_routes.get(w.unloading_node,[])
                    occupied_waiting=frozenset(q for q in dispatch.holders if dispatch.network.services[q]['kind']=='waiting')
                    for q,r in rows:
                        if time.monotonic()>deadline:
                            self.last_plan=dict(status='BUDGET_EXCEEDED',elapsed_s=time.monotonic()-started)
                            return None
                        if q in dispatch.holders or occupied_waiting.intersection(r.get('must_be_empty_nodes',[])):continue
                        options.append(dispatch._score_route(w.unloading_node,q,r))
                    parking_cache[w.unloading_node]=min(options,default=120.)
                c=Candidate(name,w,pickup,loaded,route['distance_m']/8.,parking_cache[w.unloading_node],
                            dispatch._score_route(origin,w.loading_node,route),
                            self.spans(dispatch,route,dispatch._score_route(origin,w.loading_node,route)),
                            self.spans(dispatch,onward,loaded))
                lookup[i,j]=c
                cost,_=self.evaluate(dispatch,[c],calendar)
                pair_cache[name,w.loading_node,w.unloading_node]=(c,cost)
                age=min(10.,max(0.,dispatch.now-w.release_time)/60.)
                row.append(max(0.,cost-age))
            matrix.append(row)
        selected=[lookup[i,j] for i,j in bottleneck_assignment(matrix)]
        if not selected:
            self.last_plan=dict(status='NO_FEASIBLE_CANDIDATES',elapsed_s=time.monotonic()-started)
            return None
        best,details=self.evaluate(dispatch,selected,calendar)
        if not math.isfinite(best):
            self.last_plan=dict(status='NO_JOINT_ROUTE_SLOT',snapshot_time=dispatch.now,elapsed_s=time.monotonic()-started)
            return []
        # Joint service order changes affect everyone's completion, not just
        # the new truck's independent ETA. Fixed real leases remain untouched.
        for _ in range(3):
            improved=False
            for i in range(len(selected)):
                for j in range(i+1,len(selected)):
                    if time.monotonic()>deadline:break
                    candidate=selected.copy();candidate[i],candidate[j]=candidate[j],candidate[i]
                    score,info=self.evaluate(dispatch,candidate,calendar)
                    if score+1e-6<best:selected=candidate;best=score;details=info;improved=True
                if time.monotonic()>deadline:break
            if not improved or time.monotonic()>deadline:break
        self.last_plan=dict(status='FEASIBLE',snapshot_time=dispatch.now,elapsed_s=time.monotonic()-started,
                            objective=best,opportunities=details,physical_leases_changed=False)
        pairs=[];counts={}
        for c,opportunity in zip(selected,details):
            # Road staging is not a long-term off-road buffer. Give it work
            # so the existing physical controller can clear the real source.
            staging=dispatch.network.services[dispatch.positions[c.vehicle]]['kind']=='staging'
            needed=dispatch._source_waiters(dispatch.positions[c.vehicle],c.vehicle)
            if opportunity['not_before']>dispatch.now+dispatch.scheduling.planning_interval_s and not staging and not needed:continue
            limit=dispatch.scheduling.max_loading_inflight
            existing=sum(a.operation=='LOADING' and a.work.loading_node==c.work.loading_node for a in dispatch.assignments.values())
            if limit and existing+counts.get(c.work.loading_node,0)>=limit:continue
            counts[c.work.loading_node]=counts.get(c.work.loading_node,0)+1
            pairs.append((c.work,c.vehicle,c.pickup_s))
        self.last_plan['deferred_tasks']=[c.work.task_id for c in selected if all(p[0]!=c.work for p in pairs)]
        return pairs
