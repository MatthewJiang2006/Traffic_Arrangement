"""Observed road delay for task-fixed destinations and optional off-road waits."""
from collections import defaultdict
import math
from logistics_park.common.messages import finite

class TrafficRouting:
    def __init__(self,network,model,*,update_s=.5,enter_wait_s=20.,resume_wait_s=8.,min_wait_s=5.,max_wait_s=60.,saving_s=3.):
        finite(update_s,enter_wait_s,resume_wait_s,min_wait_s,max_wait_s,saving_s)
        if not (update_s>0 and 0<=resume_wait_s<enter_wait_s and 0<=min_wait_s<=max_wait_s and saving_s>=0):
            raise ValueError('Invalid traffic routing thresholds')
        self.network=network;self.model=model;self.update_s=update_s
        self.enter_wait_s=enter_wait_s;self.resume_wait_s=resume_wait_s
        self.min_wait_s=min_wait_s;self.max_wait_s=max_wait_s;self.saving_s=saving_s
        self.observed={};self.forecasts={};self.updated_at=None;self.cache={}
        self.lengths={a:path.length for a,path in model.physical_paths.items()}
        self.capacities=model.raw['physical_capacities']
        self.roads={k:e for k,e in network.edges.items() if e['kind']=='road'}

    def observe(self,states,controller,positions,now,*,moves=None):
        finite(now)
        if self.updated_at is not None:
            if now<self.updated_at:raise ValueError('Out-of-order traffic snapshot')
            if now-self.updated_at<self.update_s:return
        observed=defaultdict(dict);forecasts={}
        for vehicle,state in states.items():
            finite(state.speed)
            ex=controller.executions[vehicle]
            if ex.finished:
                node=positions.get(vehicle)
                # Test road starts occupy their real road, not a synthetic bay.
                if node is None:continue
                if self.network.services[node]['kind']=='staging':
                    departures=[self.network.edges[k] for k in self.network.outgoing[node] if self.network.edges[k]['kind']=='road']
                    if not departures:continue
                    edge=departures[0]
                else:
                    # Some official loading/unloading points lie on a traffic
                    # lane. Keep their stopped trucks in road congestion too.
                    matches=[]
                    for alias in {e['capacity_alias'] for e in self.roads.values()}:
                        path=self.model.physical_paths.get(alias)
                        if path is None:continue
                        p=path.project(state.x,state.y)
                        angle=abs((p.heading-state.heading+math.pi)%(2*math.pi)-math.pi)
                        if p.distance<1.6 and angle<math.pi/3:matches.append((p.distance,alias))
                    if not matches:continue
                    alias=min(matches)[1];observed[alias][vehicle]=abs(state.speed);continue
            else:
                _,progress=ex.plan.project((state.x,state.y),ex.progress)
                progress=max(progress,ex.progress)
                edge=self.model.edges[ex.plan.edges[ex.plan.edge_index(progress)]]
                remaining=30.;aliases=set()
                for index in range(ex.plan.edge_index(progress),len(ex.plan.edges)):
                    part=self.model.edges[ex.plan.edges[index]]
                    if part['kind']=='road':aliases.add(part['capacity_alias'])
                    distance=ex.plan.edge_ends[index]-max(progress,ex.plan.edge_starts[index])
                    remaining-=distance/max(.1,part['speed_limit'])
                    if remaining<=0:break
                command_id=moves[vehicle].command_id if moves and vehicle in moves else None
                forecasts[vehicle]=(ex.plan.origin,tuple(ex.plan.stops),aliases,command_id)
                if edge['kind']!='road':continue
            observed[edge['capacity_alias']][vehicle]=abs(state.speed)
        self.observed=dict(observed);self.forecasts=forecasts;self.updated_at=now;self.cache.clear()

    def costs_for(self,vehicle,moves):
        # Issued itineraries contribute near-term demand, even within the same
        # observation epoch. A vehicle never pays for its own occupancy/route.
        signature=(vehicle,tuple(sorted((n,m.command_id) for n,m in moves.items())))
        if signature in self.cache:return self.cache[signature]
        forecast=defaultdict(set)
        for name,move in moves.items():
            if name==vehicle:continue
            current=self.forecasts.get(name)
            if current is not None and current[:2]==(move.origin,(move.destination,)) and current[3]==move.command_id:
                for alias in current[2]:forecast[alias].add(name)
                continue
            remaining=30.
            for key in move.route['edge_ids']:
                e=self.network.edges[key]
                if e['kind']=='road':forecast[e['capacity_alias']].add(name)
                remaining-=e['estimated_travel_time_s']
                if remaining<=0:break
        penalties={}
        for alias in set(self.observed)|set(forecast):
            cars={n:s for n,s in self.observed.get(alias,{}).items() if n!=vehicle}
            future=forecast.get(alias,set())-set(cars)
            count=len(cars);capacity=max(1,self.capacities.get(alias,1))
            demand=count+.25*len(future)
            if not demand:continue
            mean=sum(cars.values())/max(1,count)
            # Delay is distributed over the complete physical road so slicing
            # into many graph edges cannot multiply the same queue penalty.
            congestion=.5*(demand/capacity)**2
            speed_delay=max(0.,8./max(1.5,mean)-1.) if count else 0.
            penalties[alias]=(congestion+speed_delay,demand)
        costs={}
        for key,e in self.roads.items():
            alias=e['capacity_alias']
            if alias not in penalties:continue
            factor,demand=penalties[alias]
            fraction=min(1.,e['length_m']/max(.01,self.lengths.get(alias,e['length_m'])))
            costs[key]=e['estimated_travel_time_s']*factor+2.5*demand*fraction
        self.cache[signature]=costs
        return costs

    def waiting_choice(self,direct,options):
        if direct.get('congestion_delay_s',0)<self.enter_wait_s:return None
        feasible=[]
        for _,node,approach in options:
            cost=approach.get('waiting_completion_eta_s',approach['estimated_travel_time_s']+self.min_wait_s+approach['waiting_onward_freeflow_s'])
            if cost+self.saving_s<direct['estimated_travel_time_s']:feasible.append((cost,node,approach))
        return min(feasible,key=lambda row:(row[0],row[1])) if feasible else None

    def may_resume(self,route,waited_s):
        return waited_s>=self.min_wait_s and (route.get('congestion_delay_s',0)<=self.resume_wait_s or waited_s>=self.max_wait_s)
