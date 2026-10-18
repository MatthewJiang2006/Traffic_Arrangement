"""Continuous transport control independent of simulation wall-clock time."""
import math
from dataclasses import dataclass,field
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import OBB,overlaps
from logistics_park.scheduling.fcfs import FCFSScheduler,EntranceRequest
from logistics_park.scheduling.road_capacity import RoadCapacity
from logistics_park.scheduling.shared_resource import SharedResourceReservations
from logistics_park.scheduling.local_traffic import TAIL_CLEAR_M
from logistics_park.scheduling.conflict_timing import maneuver_groups
from .traffic_policy import stop_line_speed,following_speed,needs_maneuver_room
from .safety_timing import braking_distance
from .transport_routes import SteeringFeedback
from logistics_park.tasks.service_station import ServiceStation,StationConfig

BODY_LENGTH=8.344
BODY_WIDTH=2.60
BODY_OFFSET=.75  # conservative visible envelope centre ahead of rear axle
GAP=3.
MANEUVER_ROOM_HORIZON_M=40.
MANEUVER_ROOM_REACH_M=60.
ARRIVAL_POSITION_TOLERANCE_M=.35
ARRIVAL_HEADING_TOLERANCE_RAD=math.radians(5)


def stopped_at_point(state,point,heading=None):
    return (math.dist((state.x,state.y),point)<=ARRIVAL_POSITION_TOLERANCE_M
            and abs(state.speed)<.08
            and (heading is None or abs((state.heading-heading+math.pi)%(2*math.pi)-math.pi)
                 <=ARRIVAL_HEADING_TOLERANCE_RAD))


def envelope(state):
    return OBB(state.x+BODY_OFFSET*math.cos(state.heading),state.y+BODY_OFFSET*math.sin(state.heading),state.heading,BODY_LENGTH,BODY_WIDTH)


def zone_box(spec):
    x0,y0,x1,y1=spec['bounds']
    return OBB((x0+x1)/2,(y0+y1)/2,0,x1-x0,y1-y0)


@dataclass
class Execution:
    plan: object
    progress: float=0.
    checkpoint: int=0
    service_until: float | None=None
    requested: set=field(default_factory=set)
    entered: set=field(default_factory=set)
    cleared: set=field(default_factory=set)
    shared_requested: set=field(default_factory=set)
    shared_granted: set=field(default_factory=set)
    shared_entered: set=field(default_factory=set)
    shared_cleared: set=field(default_factory=set)
    started_legs: set=field(default_factory=lambda:{0})
    completed_legs: set=field(default_factory=set)
    cargo_units: int=0
    finished: bool=False
    waiting_s: float=0.
    service_wait_s: float=0.
    maximum_error: float=0.
    last_progress_time: float=0.
    unblocked_since: float|None=None
    last_progress_s: float=0.
    local_passages: set=field(default_factory=set)
    initial_station: float=0.
    steering_feedback: SteeringFeedback=field(default_factory=SteeringFeedback)


def maneuver_block_station(ego_plan,progress,leader_plan,leader_progress,*,margin=.35):
    """First ego nose station whose body enters the leader's pending sweep.

    A stopped follower inside a U-turn or reverse sweep is a geometric
    deadlock: the leader cannot take its permission while the follower's body
    is in the swept area, and the follower cannot advance because the leader
    is stationary. Its 3 m special-following gap is not the sweep boundary.
    Return the station where the ego's own body first touches the sweep, so
    the speed cap can hold the follower outside it before any body moves.
    Only pending maneuvers count; an entered one is covered by the live
    maneuver-space holds and the normal swept-body guards.
    """
    groups=[g for g in maneuver_groups(leader_plan)
            if g[2]>leader_progress and g[2]-leader_progress<=MANEUVER_ROOM_HORIZON_M
            and any(leader_plan.model.edges[leader_plan.edges[i]].get('reverse')
                    or leader_plan.model.edges[leader_plan.edges[i]]['kind']=='uturn'
                    for i in range(g[0],min(g[1]+1,len(leader_plan.edges))))]
    if not groups:return None
    entry,exit_s=groups[0][2],groups[0][3]
    count=max(1,math.ceil((exit_s-entry)/.5))
    origin=ego_plan.point(progress)
    sweep=[box for box in (envelope(VehicleState('leader-maneuver',*leader_plan.point(entry+(exit_s-entry)*i/count),
                       leader_plan.heading(entry+(exit_s-entry)*i/count))) for i in range(count+1))
           if math.dist(origin,(box.x,box.y))<=MANEUVER_ROOM_REACH_M+BODY_LENGTH]
    if not sweep:return None
    limit=min(ego_plan.length,progress+MANEUVER_ROOM_REACH_M)
    steps=max(1,math.ceil((limit-progress)/.5))
    for i in range(steps+1):
        station=progress+(limit-progress)*i/steps
        body=envelope(VehicleState('ego',*ego_plan.point(station),ego_plan.heading(station)))
        if any(overlaps(body,box,margin) for box in sweep):return station
    return None


class TransportController:
    def __init__(self,model,plans,service_s=2.,outlet_block_until=None,shared_resources=False,
                 collect_at_destination=False,retain_finished_vehicles=False,rolling_resources=False,pipeline_shared=False,local_traffic=False):
        self.collect_at_destination=collect_at_destination
        self.retain_finished_vehicles=retain_finished_vehicles
        self.pipeline_shared=pipeline_shared
        self.destination_holds={}
        self.control_denials={}
        self.local_exit_bookings={}
        self.local_traffic=None
        if local_traffic:
            from logistics_park.scheduling.local_traffic import LocalTraffic
            self.local_traffic=LocalTraffic(point_conflicts=model.raw.get('point_conflicts',False))
            shared_resources=False
        self.model=model;self.plans=plans;self.service_s=service_s
        self.executions={};self.states={};self.boxes={};self.events=[];self.frames=[]
        self.capacity=RoadCapacity(model.graph)
        self.capacity.capacities=dict(model.raw['physical_capacities'])
        self.scheduler=FCFSScheduler(tuple(model.zones),self.capacity)
        self.shared_scheduler=(SharedResourceReservations(tuple(model.shared_resources))
                               if shared_resources and model.shared_resources else None)
        if rolling_resources:
            if not self.shared_scheduler:raise ValueError('Rolling admission requires shared resources')
            from logistics_park.scheduling.rolling_resource import RollingResourceReservations
            self.shared_scheduler=RollingResourceReservations(tuple(model.shared_resources))
            self.shared_scheduler.guard=self._shared_outlet_clear
        self.zones={k:zone_box(v) for k,v in model.zones.items()}
        self.shared_zones={k:zone_box(v) for k,v in model.shared_resources.items() if 'bounds' in v} if self.shared_scheduler else {}
        self.shared_unauthorized_count=0;self.local_unauthorized_count=0
        self.services={key:ServiceStation(StationConfig(key,tuple(spec['berth_nodes']),
            service_capacity=1,waiting_capacity=0,service_time=spec.get('service_s',5)))
            for key,spec in model.raw.get('service_stations',{}).items()}
        self.berth_zones={key:zone_box(spec) for key,spec in model.raw.get('service_stations',{}).items()}
        self.collision_count=0;self.unauthorized_count=0;self.capacity_violation_steps=0
        self.maximum_active=0;self.maximum_moving=0;self.maximum_queue=0;self.maximum_error=0.
        self.now=0.;self.last_tick=0.
        self.outlet_block_until=outlet_block_until or {};self.reopened=set()
        for resource,end in self.outlet_block_until.items():
            if resource not in model.zones or end<=0:raise ValueError('invalid outlet blocking fixture')
            self.events.append({'time':0.,'event':'OUTLET_CLOSED','resource':resource,'until_s':end,'test_fixture':True})

    def can_activate(self,name):
        plan=self.plans[name];p=plan.point(0);heading=plan.heading(0)
        candidate=envelope(VehicleState(name,*p,heading))
        if any(overlaps(candidate,b,.25) for n,boxes in self.boxes.items() for b in boxes if not self.executions[n].finished):return False
        for index,v in enumerate(plan.visits):
            if v.start>8:break
            if overlaps(candidate,self.zones[v.resource],.1) and v.resource in self.scheduler.owners:return False
        return True

    def activate(self,name,now):
        if name in self.executions:raise ValueError('vehicle activated twice')
        self.executions[name]=Execution(self.plans[name],last_progress_time=now)
        self.events.append({'time':now,'vehicle_id':name,'event':'STARTED','leg':0,'origin':self.plans[name].origin,'destination':self.plans[name].stops[0]})
        first=self.plans[name].checkpoints[0] if self.plans[name].checkpoints else {}
        if first.get('operation')=='LOADING' and 'cycle' in first:
            self.events.append({'time':now,'vehicle_id':name,'event':'TRANSPORT_TASK_RELEASED','cycle':first['cycle'],'source':first['site']})

    def continue_route(self,name,plan,now,*,state):
        """Replace a stopped leg without deleting the truck or its road leases."""
        if not self.retain_finished_vehicles or name not in self.executions or not self.executions[name].finished:
            raise ValueError('Only a retained finished leg may continue')
        if abs(state.speed)>.1 or math.dist((state.x,state.y),plan.point(0))>.5:
            raise ValueError('Next leg must start at the actual stopped pose')
        if abs((state.heading-plan.heading(0)+math.pi)%(2*math.pi)-math.pi)>.35:
            raise ValueError('Next leg has incompatible heading')
        owned={k for k,v in self.shared_scheduler.owners.items() if v==name} if self.shared_scheduler else set()
        required={v.channel_id for v in plan.shared_visits}
        if owned-required:
            raise ValueError('Next leg omits retained shared-resource leases')
        self.plans[name]=plan;self.executions[name]=Execution(plan,last_progress_time=now)
        # A stopped arrival may be a few centimetres ahead of the canonical
        # source. Audit movement beyond that measured starting pose, rather
        # than interpreting the leg switch itself as an unauthorized entry.
        self.executions[name].initial_station=plan.project((state.x,state.y),0.,behind=0,ahead=.5)[1]
        self.events.append(dict(time=now,event='LEG_CONTINUED',vehicle_id=name,origin=plan.origin,destination=plan.stops[0],retained_resources=sorted(owned)))

    def reroute_stopped(self,name,plan,now,*,state):
        """Change only the future of a stopped road leg with no live passage."""
        self._reroute_road(name,plan,now,state=state,moving=False)

    def reroute_forward(self,name,plan,now,*,state):
        """Redirect before a junction, preserving the entire braking prefix."""
        if not self.local_traffic or not self.local_traffic.proactive_admission or state.speed<-.08:
            raise ValueError('Moving diversion requires proactive forward admission')
        self._reroute_road(name,plan,now,state=state,moving=True)

    def _reroute_road(self,name,plan,now,*,state,moving):
        ex=self.executions[name]
        if (not self.local_traffic or ex.finished or (not moving and abs(state.speed)>.08)
                or any(key[0]==name for key in self.local_traffic.passages)):
            raise ValueError('Diversion requires a stopped, uncommitted road vehicle')
        if self.model.edges[ex.plan.edges[ex.plan.edge_index(ex.progress)]]['kind']!='road':
            raise ValueError('Diversion must start on an ordinary road')
        # Preserve all traversed geometry and the complete nose of the actual
        # stopped body. An unentered turn beyond its stop line may change.
        limit=ex.progress+max(6.,braking_distance(state.speed))
        # A nearby new access can merge with a junction already traversed.
        # Identical prefix points alone do not preserve the admission contract:
        # the merged action would require a lease whose entry is behind us.
        # No passage is live here, so reject that diversion before changing
        # either the active plan or its capacity commitments.
        if hasattr(plan,'edge_starts') and any(
                entry+.2<ex.progress<exit_s+.2
                for _,_,entry,exit_s in maneuver_groups(plan)):
            raise ValueError('Diversion retroactively requires a traversed maneuver passage')
        if moving:
            braking_end=ex.progress+braking_distance(state.speed)
            if hasattr(plan,"edge_starts") and any(g[3]>ex.progress and g[2]-7.8<braking_end for g in maneuver_groups(plan)):
                raise ValueError('Diversion cannot brake before a newly required action stop line')
        tolerance=.001 if getattr(self.model,'raw',{}).get('point_conflicts',False) else 1e-6
        if plan.length<=limit:raise ValueError('Diversion omits the retained road prefix')
        for i in range(math.ceil(limit/.25)+1):
            station=min(limit,i*.25)
            if (math.dist(plan.point(station),ex.plan.point(station))>tolerance
                    or plan.reverse(station)!=ex.plan.reverse(station)):
                raise ValueError('Diversion changes the traversed or committed prefix')
        if math.dist((state.x,state.y),plan.point(ex.progress))>.5:
            raise ValueError('Diversion must preserve the actual stopped pose')
        if getattr(self.local_traffic,'check_diversion_room',False):
            self._check_diversion_reverse_room(name,plan,ex.progress,state)
        ex.plan=plan;self.plans[name]=plan
        # No live maneuver remains. Cancel only unentered future capacity;
        # actual road occupancy is retained and remeasured every tick.
        for alias in self.capacity.committed:self.capacity.release_commitment(name,alias)
        self.events.append(dict(time=now,event='PROACTIVE_ROAD_DIVERSION' if moving else 'STOPPED_ROAD_DIVERSION',vehicle_id=name,destination=plan.stops[0],progress_m=ex.progress,retained_prefix_end_m=limit))

    def _check_diversion_reverse_room(self,name,plan,progress,state):
        """Do not divert a lane leader into its stopped follower's body.

        A follower cannot clear a reverse entrance by driving through its
        leader. Reject the proposed route before replacing the old plan or
        releasing capacity, so queue clearance can try another waiting site.
        Only nearby new reverse maneuvers need this check; distant traffic
        remains governed by the normal passage admission checks.
        """
        if not hasattr(plan,'edge_starts'):return
        followers=[]
        for other,pose in getattr(self,'states',{}).items():
            if other==name or abs(pose.speed)>.08:continue
            dx,dy=pose.x-state.x,pose.y-state.y
            along=dx*math.cos(state.heading)+dy*math.sin(state.heading)
            across=-dx*math.sin(state.heading)+dy*math.cos(state.heading)
            aligned=math.cos(pose.heading-state.heading)>.95
            if aligned and -25.<along<0. and abs(across)<BODY_WIDTH:
                followers.extend(getattr(self,'boxes',{}).get(other,[envelope(pose)]))
        if not followers:return
        for start,end,entry,exit_s in maneuver_groups(plan):
            if not (progress<exit_s and entry<=progress+25.):continue
            if not any(plan.model.edges[plan.edges[i]].get('reverse') for i in range(start,end+1)):continue
            lo=max(progress,entry);count=max(1,math.ceil((exit_s-lo)/.25))
            for i in range(count+1):
                s=lo+(exit_s-lo)*i/count
                body=envelope(VehicleState(name,*plan.point(s),plan.heading(s)))
                if any(overlaps(body,other,self.local_traffic.margin_m) for other in followers):
                    raise ValueError('Diversion reverse entrance is occupied by a stopped lane follower')

    def _outlet_clear(self,edge,name):
        ex=self.executions[name]
        owner_ids={j for j,g in self.scheduler.owners.items() if g.request.vehicle_id==name}
        visit=next((v for i,v in enumerate(ex.plan.visits) if i in ex.requested and i not in ex.cleared
                    and v.outgoing==edge and v.resource not in owner_ids),None)
        if visit is None:return False
        if self.shared_scheduler and any(i not in ex.shared_cleared and
                shared.start <= visit.end+BODY_LENGTH+GAP and shared.end >= visit.start and
                not self.shared_scheduler.allows(shared.channel_id,name)
                for i,shared in enumerate(ex.plan.shared_visits)):
            return False
        if self.now<self.outlet_block_until.get(visit.resource,0.):return False
        # A lane-change zone includes straight traffic; never grant it around
        # a vehicle whose tail still occupies the merge area.
        if self.model.zones[visit.resource]['kind']=='lane_change':
            if any(overlaps(b,self.zones[visit.resource],.1) for n,boxes in self.boxes.items() if n!=name
                   and (self.retain_finished_vehicles or not self.executions[n].finished) for b in boxes):return False
        outlet=ex.plan.point(min(ex.plan.length,visit.end+1.))
        heading=ex.plan.heading(min(ex.plan.length,visit.end+1.));co,si=math.cos(heading),math.sin(heading)
        for n,state in self.states.items():
            if n==name or (self.executions[n].finished and not self.retain_finished_vehicles):continue
            dx,dy=state.x-outlet[0],state.y-outlet[1]
            longitudinal=dx*co+dy*si;lateral=abs(-dx*si+dy*co)
            if lateral<BODY_WIDTH+.2 and -4.5<longitudinal<BODY_LENGTH+GAP:return False
        return True

    def _shared_outlet_clear(self,request):
        """Do not commit a corridor whose measured exit is obstructed."""
        if request.vehicle_id not in self.executions:return True  # observed parked attach
        ex=self.executions[request.vehicle_id]
        visits=[v for i,v in enumerate(ex.plan.shared_visits) if v.channel_id in request.channels and i not in ex.shared_cleared]
        if not visits:return False
        last=max(visits,key=lambda v:v.end)
        exit_s=min(ex.plan.length,last.end+BODY_LENGTH+GAP)
        candidate=envelope(VehicleState(request.vehicle_id,*ex.plan.point(exit_s),ex.plan.heading(exit_s)))
        return not any(overlaps(candidate,b,.5) for n,boxes in self.boxes.items() if n!=request.vehicle_id for b in boxes)

    def _leader_before_shared(self,name,ex,visit):
        ahead=max(0.,visit.start-ex.progress)+BODY_LENGTH+GAP
        origin=ex.plan.point(ex.progress)
        for other,state in self.states.items():
            if other==name or math.dist(origin,(state.x,state.y))>ahead+3.:continue
            error,s=ex.plan.project((state.x,state.y),ex.progress,behind=0,ahead=ahead)
            angle=abs((state.heading-ex.plan.heading(s)+math.pi)%(2*math.pi)-math.pi)
            if error<1.4 and angle<math.pi/3 and ex.progress+.1<s<=ex.progress+ahead:return other
        return None

    def _stationary_obstacle_distance(self,name,state,ex):
        if not hasattr(self,'control_denials'):self.control_denials={}
        # A turning truck's nose can hit a stopped leader even when that
        # leader's axle is no longer close to the turning centerline.
        ahead=max(12.,braking_distance(state.speed,3.))
        origin=ex.plan.point(ex.progress)
        obstacles=[(other,b) for other,boxes in self.boxes.items() if other!=name
                   and abs(self.states[other].speed)<.1
                   and math.dist(origin,(self.states[other].x,self.states[other].y))<ahead+BODY_LENGTH+3.
                   for b in boxes]
        if not obstacles:return None
        end=min(ex.plan.length,ex.progress+ahead)
        count=max(1,math.ceil((end-ex.progress)/.5))
        planned_heading=ex.plan.heading(ex.progress)
        yaw_error=(state.heading-planned_heading+math.pi)%(2*math.pi)-math.pi
        for i in range(count+1):
            s=ex.progress+(end-ex.progress)*i/count
            point=ex.plan.point(s);heading=ex.plan.heading(s)
            if getattr(getattr(self,'model',None),'raw',{}).get('point_conflicts',False):
                # Anchor the immediate prediction to the observed truck. At a
                # gear boundary the polyline tangent can differ from its yaw;
                # a projected overlap must not masquerade as current contact.
                weight=max(0.,1.-(s-ex.progress)/3.)
                point=(point[0]+weight*(state.x-origin[0]),point[1]+weight*(state.y-origin[1]))
                heading+=weight*yaw_error
            body=envelope(VehicleState(name,*point,heading))
            for other,b in obstacles:
                if overlaps(body,b,.2):
                    distance=max(0.,s-ex.progress-.75)
                    self.control_denials.setdefault(name,[]).append(dict(blocked_by=other,reason='stationary_swept_body',distance_m=distance,conflict_s=s))
                    return distance
        return None

    def _passage_capacity_aliases(self,plan,start,outlet_s):
        """Book the current action's one-car bottlenecks and road outlet."""
        aliases=set()
        for j in range(start,min(len(plan.edges),plan.edge_index(outlet_s)+1)):
            key=plan.edges[j];alias=self.model.aliases[key]
            if self.model.edges[key]['kind']=='road' or self.capacity.capacities[alias]==1:
                aliases.add(alias)
        return aliases

    def _parking_capacity_predecessor(self,name,ex,start,outlet_s):
        # Defer NEW bookings in mixed transport/parking queues too.
        # Every entered/held lease remains protected by the caller.
        parking_enabled=getattr(self,'parking_entry_priority',False)
        transport_enabled=getattr(self,'transport_entry_priority',False)
        if not (parking_enabled or transport_enabled):return None
        parking=getattr(self,'parking_moves',())
        aliases={}
        for i in range(start,min(len(ex.plan.edges),ex.plan.edge_index(outlet_s)+1)):
            key=ex.plan.edges[i];alias=self.model.aliases[key]
            if (self.model.edges[key]['kind']=='junction' and self.capacity.capacities[alias]==1
                    and ex.plan.edge_starts[i]>=ex.plan.edge_starts[start]+12.):
                aliases.setdefault(alias,ex.plan.edge_starts[i])
        if not aliases:return None
        from logistics_park.scheduling.conflict_timing import arrival_profile
        cache=getattr(self,'parking_profile_cache',None)
        if cache is None or cache[0]!=self.now:
            cache=(self.now,{});self.parking_profile_cache=cache
        def profile(vehicle,execution):
            if vehicle not in cache[1]:
                cache[1][vehicle]=arrival_profile(execution,self.states[vehicle],self.now,
                    hold=self.destination_holds.get(vehicle),dt=.5,limit_s=120.)
            return cache[1][vehicle]
        own=None
        for other in sorted(self.executions):
            if other==name or other not in self.states:continue
            if name in parking or other in parking:
                if not parking_enabled:continue
            else:
                if not transport_enabled:continue
                # A physically forward truck on this or an adjacent parallel
                # lane must clear the common junction before a later merge
                # books it. Crossing/opposing traffic retains normal timing.
                state=self.states[name];front=self.states[other]
                dx,dy=front.x-state.x,front.y-state.y
                co,si=math.cos(state.heading),math.sin(state.heading)
                forward=dx*co+dy*si;lateral=abs(-dx*si+dy*co)
                angle=abs((front.heading-state.heading+math.pi)%(2*math.pi)-math.pi)
                if not (.1<forward<80. and lateral<4.2 and angle<math.pi/3):continue
            leader=self.executions[other]
            if leader.finished:continue
            # A downstream ETA preference cannot make a physical lane leader
            # wait for its own follower. The follower cannot reach that exit
            # until this truck clears; ordinary passage/body guards still apply.
            if name in parking or other in parking:
                state=self.states[name]
                error,station=leader.plan.project((state.x,state.y),leader.progress,
                    behind=0.,ahead=80.)
                angle=abs((state.heading-leader.plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
                if error<1.4 and angle<math.pi/3 and station>leader.progress+.1:
                    continue
            shared={}
            for i,key in enumerate(leader.plan.edges):
                alias=self.model.aliases[key]
                if alias in aliases and leader.plan.edge_ends[i]+TAIL_CLEAR_M>=leader.progress:
                    shared.setdefault(alias,leader.plan.edge_starts[i])
            if not shared:continue
            alias=min(shared,key=lambda a:aliases[a])
            # Do not impose pair priorities with opposite resource orders.
            if alias!=min(shared,key=lambda a:shared[a]):continue
            if own is None:own=profile(name,ex)
            eta=profile(other,leader).eta(shared[alias])
            if math.isfinite(eta) and eta<=60. and eta+2.<own.eta(aliases[alias]):
                return other,alias
        return None

    def _release_unused_local_bookings(self):
        for (name,_),lease in self.local_traffic.released:
            outlet=lease.get('outlet',min(lease['plan'].length,lease['end']+BODY_LENGTH+GAP))
            for key,(plan,station) in list(self.local_exit_bookings.items()):
                if key[0]!=name or plan is not lease['plan'] or abs(station-outlet)>1e-6:continue
                if any(k[0]==name and v['plan'] is plan and abs(v.get('outlet',min(plan.length,v['end']+BODY_LENGTH+GAP))-station)<1e-6
                        for k,v in self.local_traffic.passages.items()):continue
                self.capacity.release_commitment(name,key[1]);del self.local_exit_bookings[key]
        self.local_traffic.released.clear()

    def tick(self,states,now,body_boxes=None):
        dt=max(0,now-self.last_tick);self.last_tick=now;self.now=now
        for resource,end in self.outlet_block_until.items():
            if now>=end and resource not in self.reopened:
                self.reopened.add(resource)
                self.events.append({'time':now,'event':'OUTLET_REOPENED','resource':resource,'test_fixture':True})
        self.states={n:s for n,s in states.items() if n in self.executions and (self.retain_finished_vehicles or not self.executions[n].finished)}
        self.boxes={n:(body_boxes[n] if body_boxes else [envelope(s)]) for n,s in self.states.items()}
        for key,station in self.services.items():
            for n,admission in list(station.occupants.items()):
                if (admission.service_completed and n in self.boxes
                        and not any(overlaps(b,self.berth_zones[key],.1) for b in self.boxes[n])):
                    station.release(n)
                    self.events.append({'time':now,'vehicle_id':n,'event':'BERTH_RELEASED','site':key,'tail_clear':True})
        occupied={}
        for n,state in self.states.items():
            ex=self.executions[n]
            cp=ex.plan.checkpoints[ex.checkpoint] if ex.checkpoint<len(ex.plan.checkpoints) else None
            stop_s=cp['s'] if cp and cp['event'] in ('ARRIVAL','GEAR_CHANGE') else None
            ahead=max(3.,abs(state.speed)*dt+2.) if self.model.raw.get('point_conflicts',False) else 3.
            projection_options=dict(stop_s=stop_s,ahead=ahead)
            gear_changes=[c['s'] for c in ex.plan.checkpoints[:ex.checkpoint] if c['event']=='GEAR_CHANGE']
            if gear_changes:projection_options['start_s']=gear_changes[-1]
            error,s=ex.plan.project((state.x,state.y),ex.progress,**projection_options)
            if cp and cp['event']=='GEAR_CHANGE':
                # At the boundary edge_index selects the next gear. Keep the
                # incoming steering/direction while braking; the stopped
                # checkpoint below is the only operation that commits a gear.
                s=min(s,max(0.,cp['s']-1e-6))
            ex.progress=max(ex.progress,s);ex.maximum_error=max(ex.maximum_error,error);self.maximum_error=max(self.maximum_error,error)
            active_edge=ex.plan.edges[ex.plan.edge_index(ex.progress)]
            tolerance=self.model.edges[active_edge].get('maximum_path_error_m',.7)
            if error>tolerance:raise RuntimeError(n+' path deviation '+str(round(error,3)))
            if s>ex.last_progress_s+.2:ex.last_progress_s=s;ex.last_progress_time=now
            index=ex.plan.edge_index(s);edge=ex.plan.edges[index]
            occupied[n]=VehicleState(n,state.x,state.y,state.heading,speed=state.speed,
                edge_id=self.model.aliases[edge],s=max(0,s-ex.plan.edge_starts[index]))
        if self.local_traffic:self._release_passed_commitments()
        self.capacity.update(occupied)
        if self.shared_scheduler:
            for n in self.states:
                ex=self.executions[n]
                if ex.finished:continue
                next_shared=min((i for i in range(len(ex.plan.shared_visits))
                                 if i not in ex.shared_cleared),
                                key=lambda i:ex.plan.shared_visits[i].start,default=None)
                for i, visit in enumerate(ex.plan.shared_visits):
                    if i in ex.shared_cleared:
                        continue
                    if any(j not in ex.shared_cleared and v.channel_id==visit.channel_id
                           for j,v in enumerate(ex.plan.shared_visits[:i])):
                        continue
                    ahead=self.model.shared_resources[visit.channel_id].get('request_ahead_m',40)
                    if (i not in ex.shared_entered and visit.start-ex.progress < ahead
                            and (not self.collect_at_destination or self.pipeline_shared or i==next_shared)):
                        # Shared FIFO must respect the actual lane leader,
                        # including one that merged ahead after a request.
                        # Only an unentered, physically clear lease can yield.
                        leader=self._leader_before_shared(n,ex,visit) if self.pipeline_shared else None
                        if leader is not None and not hasattr(self.shared_scheduler,'request_many'):
                            area=self.shared_zones.get(visit.channel_id)
                            if i not in ex.shared_entered and area is not None and not any(overlaps(b,area,.1) for b in self.boxes[n]):
                                if self.shared_scheduler.allows(visit.channel_id,n):
                                    self.shared_scheduler.release(visit.channel_id,n,now,tail_clear=True)
                                    self.events.append(dict(time=now,event='UNENTERED_SHARED_YIELD',vehicle_id=n,leader=leader,channel_id=visit.channel_id))
                                else:self.shared_scheduler.cancel_request(visit.channel_id,n,now)
                                ex.shared_requested.discard(i);ex.shared_granted.discard(i)
                                continue
                        if hasattr(self.shared_scheduler,'request_many'):
                            # Nearby regions must be acquired together before
                            # entering: a full truck cannot wait between them.
                            bundle=[(i,visit)];end=visit.end
                            later=sorted(((j,v) for j,v in enumerate(ex.plan.shared_visits) if j not in ex.shared_cleared and v.start>=visit.start and j!=i),key=lambda pair:pair[1].start)
                            for j,v in later:
                                if v.start>end+BODY_LENGTH+GAP:break
                                bundle.append((j,v));end=max(end,v.end)
                            channels=tuple(v.channel_id for j,v in bundle)
                            duration=max(1.,(end-visit.start+BODY_LENGTH+GAP)/max(.8,ex.plan.speed_cap(visit.start)))
                            granted=self.shared_scheduler.request_many(channels,n,now,predicted_s=duration)
                            ex.shared_requested.update(j for j,v in bundle)
                        else:
                            granted=self.shared_scheduler.request(visit.channel_id,n,now)
                        ex.shared_requested.add(i)
                        if granted and i not in ex.shared_granted:
                            ex.shared_granted.add(i)
                            self.events.append({'time':now,'vehicle_id':n,
                                'event':'SHARED_RESOURCE_GRANTED','channel_id':visit.channel_id})
                    # Use every measured body box when the channel has physical
                    # bounds. Otherwise all footprint corners must pass the
                    # route boundary; axle progress alone cannot release it.
                    area=self.shared_zones.get(visit.channel_id)
                    if (self.collect_at_destination and i not in ex.shared_entered
                            and ex.progress > visit.end + .5 and area is not None
                            and self.shared_scheduler.allows(visit.channel_id,n)
                            and not any(overlaps(b,area,.1) for b in self.boxes[n])):
                        if self.shared_scheduler.release(visit.channel_id,n,now,tail_clear=True):
                            ex.shared_cleared.add(i)
                            self.events.append({'time':now,'vehicle_id':n,
                                'event':'UNUSED_SHARED_RESERVATION_RELEASED','channel_id':visit.channel_id})
                    if i in ex.shared_entered and (area is not None or ex.progress >= visit.end-.2):
                        tail_clear=(not any(overlaps(b,area,.1) for b in self.boxes[n]) if area else
                                    all(ex.plan.project(c,ex.progress,behind=10,ahead=10)[1]>=visit.end-.2
                                        for b in self.boxes[n] for c in b.corners(.1)))
                        if tail_clear and self.shared_scheduler.release(visit.channel_id,n,now,tail_clear=True):
                            ex.shared_cleared.add(i)
                            self.events.append({'time':now,'vehicle_id':n,
                                'event':'SHARED_RESOURCE_RELEASED','channel_id':visit.channel_id,'tail_clear':True})
        for n,state in self.states.items():
            ex=self.executions[n]
            for i,visit in enumerate(ex.plan.visits):
                if i in ex.cleared:continue
                previous=any(j not in ex.cleared and v.resource==visit.resource for j,v in enumerate(ex.plan.visits[:i]))
                if previous:continue
                if visit.start-ex.progress<20 and i not in ex.entered:
                    # FCFS lane leaders must use a common physical road station,
                    # never each truck's unrelated total itinerary progress.
                    lane_s=self.model.physical_paths[visit.incoming].project(state.x,state.y).s
                    self.scheduler.request(EntranceRequest(n,visit.resource,visit.incoming,visit.connector,visit.outgoing,now,lane_s))
                    ex.requested.add(i)
                if i in ex.entered and ex.progress>visit.end and not any(overlaps(b,self.zones[visit.resource],.1) for b in self.boxes[n]):
                    self.scheduler.complete(visit.resource,n,now);ex.cleared.add(i)
                    self.capacity.release_commitment(n,visit.outgoing)
        self.scheduler.advance(now,self._outlet_clear)
        for n,state in self.states.items():
            ex=self.executions[n]
            for i,visit in enumerate(ex.plan.visits):
                if i in ex.cleared or ex.progress<visit.start-9 or ex.progress>visit.end+12:continue
                inside=any(overlaps(b,self.zones[visit.resource]) for b in self.boxes[n])
                if inside:
                    if not self.scheduler.allows(n,visit.resource,visit.connector):
                        self.unauthorized_count+=1;raise RuntimeError(n+' unauthorized entry '+visit.resource)
                    if i not in ex.entered:self.scheduler.mark_entered(visit.resource,n,now);ex.entered.add(i)
        if self.shared_scheduler:
            for channel,area in self.shared_zones.items():
                for n,boxes in self.boxes.items():
                    if any(overlaps(b,area) for b in boxes) and not self.shared_scheduler.allows(channel,n):
                        self.shared_unauthorized_count+=1
                        raise RuntimeError(n+' unauthorized shared entry '+channel)
        names=list(self.states)
        for i,n in enumerate(names):
            for m in names[i+1:]:
                if any(overlaps(a,b,.05) for a in self.boxes[n] for b in self.boxes[m]):
                    self.collision_count+=1;raise RuntimeError('vehicle envelope overlap '+n+' '+m)
        if self.capacity.violations:
            self.capacity_violation_steps+=1;raise RuntimeError('physical road capacity exceeded')
        if self.local_traffic:
            self.local_traffic.action_groups={n:maneuver_groups(ex.plan) for n,ex in self.executions.items() if not ex.finished}
            for n,groups in self.local_traffic.action_groups.items():
                ex=self.executions[n]
                for start,end,entry,exit_s in groups:
                    audit_entry=max(entry,ex.initial_station) if start==0 else entry
                    if not audit_entry+.2<ex.progress<exit_s+.2:continue
                    lease=self.local_traffic.passages.get((n,start))
                    if not lease or lease['plan'] is not ex.plan or lease['end']<exit_s-1e-6:
                        self.local_unauthorized_count+=1
                        raise RuntimeError(n+' actual local entry without a live passage '+str(start))
            self.local_traffic.prepare(self.states,self.executions,self.destination_holds,envelope,now)
            self.local_traffic.timing.prepare(self.states,self.executions,self.local_traffic.action_groups,self.destination_holds,self.local_traffic,envelope,now)
            self._release_unused_local_bookings()
        if self.local_traffic:self.local_traffic.committed=self.capacity.committed
        commands={};self.control_denials={};self.control_trace={}
        order=self.local_traffic.timing.order if self.local_traffic else []
        for n in order+[n for n in self.states if n not in order]:
            state=self.states[n];ex=self.executions[n]
            if ex.finished:
                commands[n]=(0.,0.);continue
            s=ex.progress;target=ex.plan.speed_cap(s);waiting=bool(self.local_traffic and n in self.local_traffic.wait_since);service_waiting=False
            if (ex.checkpoint and ex.plan.checkpoints[ex.checkpoint-1]['event']=='GEAR_CHANGE'
                    and s-ex.plan.checkpoints[ex.checkpoint-1]['s']<.25):
                target=ex.plan.speed_cap(s+.25)
            trace=[];self.control_trace[n]=trace
            def record(stage):
                trace.append(dict(stage=stage,target_m_s=target))
            record('route_speed')
            alignment_cap=getattr(ex.plan,'curve_exit_alignment_speed_cap',None)
            if alignment_cap:
                target=min(target,alignment_cap((state.x,state.y),state.heading,s))
                record('curve_exit_alignment')
            clearance=self.local_traffic.maneuver_holds.get(n) if self.local_traffic else None
            if clearance:
                distance=max(0.,clearance['stop_s']-s)
                target=min(target,stop_line_speed(distance));waiting=True
                self.control_denials.setdefault(n,[]).append(dict(clearance,distance_m=distance))
            yield_hold=self.local_traffic.yield_holds.get(n) if self.local_traffic else None
            if yield_hold:
                distance=max(0.,yield_hold['stop_s']-s)
                target=min(target,stop_line_speed(distance));waiting=True
                self.control_denials.setdefault(n,[]).append(dict(yield_hold,distance_m=distance))
            if self.local_traffic and self.local_traffic.clearance_pauses.get(n):
                target=0.;waiting=True
                for other in sorted(self.local_traffic.clearance_pauses[n]):
                    self.control_denials.setdefault(n,[]).append(dict(blocked_by=other,reason='clearance_priority',distance_m=0.))
            record('maneuver_clearance')
            if self.local_traffic:
                for start,end,entry,exit_s in self.local_traffic.action_groups.get(n,()):
                    index=end+1
                    if exit_s+TAIL_CLEAR_M<s or entry-s>max(25.,abs(state.speed)**2/1.2+15.):continue
                    lease=self.local_traffic.passages.get((n,start))
                    if lease and lease['end']>=exit_s-1e-6:continue
                    _,outlet_s=self.local_traffic.passage_outlet(ex,start,end)
                    outgoing=self._passage_capacity_aliases(ex.plan,start,outlet_s)
                    saturated=[a for a in outgoing if n not in self.capacity.occupants[a]
                        and n not in self.capacity.committed[a] and self.capacity.road_state(a).is_saturated]
                    predecessor=self._parking_capacity_predecessor(n,ex,start,outlet_s)
                    room=not saturated and predecessor is None
                    if (room and self.destination_holds.get(n,ex.plan.length)>=outlet_s
                            and self.local_traffic.claim_passage(n,ex,start,index-1,self.states,self.boxes,envelope)):
                        self._release_unused_local_bookings()
                        for alias in outgoing:
                            if not self.capacity.reserve(n,alias):raise RuntimeError('Non-atomic local exit capacity')
                            self.local_exit_bookings[(n,alias)]=(ex.plan,outlet_s)
                        ex.local_passages.add(start)
                    else:
                        distance=max(0.,entry-s-7.8)
                        if predecessor:
                            self.control_denials.setdefault(n,[]).append(dict(blocked_by=predecessor[0],
                                reason='parking_future_entry_priority',resource=predecessor[1],start=start,distance_m=distance))
                        for alias in sorted(saturated):
                            for other in sorted(self.capacity.occupants[alias]|self.capacity.committed[alias]):
                                self.control_denials.setdefault(n,[]).append(dict(blocked_by=other,
                                    reason='passage_capacity',resource=alias,start=start,distance_m=distance))
                        target=min(target,stop_line_speed(distance) if self.model.raw.get('stop_at_conflict_line',False) else min(math.sqrt(1.2*distance),distance*.65));waiting=True
                window=self.local_traffic.timing.windows.get(n)
                if window and not self.model.raw.get('stop_at_conflict_line',False) and (n,window['start']) not in self.local_traffic.passages:
                    target=self.local_traffic.timing.rolling_cap(n,target)
            record('local_passage_and_timing')
            if n in self.destination_holds:
                distance=max(0.,self.destination_holds[n]-s)
                target=min(target,stop_line_speed(distance) if self.model.raw.get('stop_at_conflict_line',False) else min(math.sqrt(1.2*distance),distance*.65));waiting=True
            record('destination_hold')
            if self.shared_scheduler:
                for i, visit in enumerate(ex.plan.shared_visits):
                    spec=self.model.shared_resources[visit.channel_id]
                    if i in ex.shared_cleared or i in ex.shared_entered or visit.start-s > spec.get('approach_ahead_m',25):
                        continue
                    if any(j not in ex.shared_cleared and v.channel_id==visit.channel_id
                           for j,v in enumerate(ex.plan.shared_visits[:i])):
                        continue
                    if not self.shared_scheduler.allows(visit.channel_id,n):
                        distance=max(0,visit.start-s-spec.get('stop_before_m',7.8))
                        target=min(target,math.sqrt(1.2*distance),distance*.65)
                        waiting=True
                        if abs(state.speed)<.08:
                            self.events.append({'time':now,'vehicle_id':n,
                                'event':'SHARED_RESOURCE_WAITING','channel_id':visit.channel_id})
                    else:
                        area=self.shared_zones.get(visit.channel_id)
                        inside=(any(overlaps(b,area) for b in self.boxes[n]) if area else s >= visit.start-.2)
                        if not inside:
                            continue
                        ex.shared_entered.add(i)
                        self.events.append({'time':now,'vehicle_id':n,
                            'event':'SHARED_RESOURCE_ENTERED','channel_id':visit.channel_id})
            record('shared_permissions')
            for i,v in enumerate(ex.plan.visits):
                if i in ex.cleared or i in ex.entered or v.start<s-10:continue
                if v.start-s>25:break
                if not self.scheduler.allows(n,v.resource,v.connector):
                    # Largest front support of the conservative body envelope
                    # in any orientation, plus a stop-line buffer.
                    distance=max(0,v.start-s-7.8)
                    target=min(target,stop_line_speed(distance) if self.model.raw.get('stop_at_conflict_line',False) else min(math.sqrt(1.2*distance),distance*.65));waiting=True
            record('junction_permissions')
            # Project only onto the upcoming route window. Repeated roundtrip
            # geometry must not jump back to an earlier traversal of the road.
            follow_origin=ex.plan.point(s)
            follow_ahead=max(35.,abs(state.speed)**2/1.2+BODY_LENGTH+GAP+5.)
            for m,other in self.states.items():
                if m==n:continue
                # Include braking distance at the current speed. The arc
                # window plus segment/lateral tolerance bounds this filter.
                if math.dist(follow_origin,(other.x,other.y))>follow_ahead+3.:continue
                error,leader=ex.plan.project((other.x,other.y),s,behind=0,ahead=follow_ahead)
                heading_error=abs((other.heading-ex.plan.heading(leader)+math.pi)%(2*math.pi)-math.pi)
                if error<1.4 and heading_error<math.pi/3 and leader>s+.1:
                    blocked=maneuver_block_station(ex.plan,s,self.executions[m].plan,self.executions[m].progress)
                    if blocked is not None:leader=min(leader,blocked+BODY_LENGTH+GAP)
                    if self.model.raw.get('compact_following',False):
                        cap=following_speed(leader-s,other.speed,special=needs_maneuver_room(self.executions[m]))
                        target=min(target,cap);waiting=waiting or cap<ex.plan.speed_cap(s)
                    else:
                        distance=max(0,leader-s-BODY_LENGTH-GAP)
                        target=min(target,math.sqrt(1.2*distance),distance/1.5);waiting=True
            record('following')
            obstacle_distance=self._stationary_obstacle_distance(n,state,ex)
            if obstacle_distance is not None:
                target=min(target,math.sqrt(1.2*obstacle_distance),obstacle_distance*.65);waiting=True
            record('stationary_body')
            if ex.checkpoint<len(ex.plan.checkpoints):
                checkpoint=ex.plan.checkpoints[ex.checkpoint]
                if checkpoint['event']=='ARRIVAL':
                    station=self.services.get(checkpoint['site'])
                    remaining=max(0,checkpoint['s']-s)
                    target=min(target,math.sqrt(1.2*remaining),remaining*.8)
                    if station and n not in station.occupants and remaining<20:
                        channel=checkpoint.get('channel_id')
                        allowed=(not channel or bool(self.shared_scheduler and self.shared_scheduler.allows(channel,n)))
                        if allowed:
                            admission=station.reserve(n,str(checkpoint['leg']))
                            if admission is not None:
                                self.events.append({'time':now,'vehicle_id':n,'event':'BERTH_RESERVED','site':checkpoint['site']})
                        if n not in station.occupants:
                            distance=max(0,remaining-BODY_LENGTH-GAP)
                            target=min(target,stop_line_speed(distance) if self.model.raw.get('stop_at_conflict_line',False) else min(math.sqrt(1.2*distance),distance*.65));waiting=True
                    if ex.service_until is not None:
                        target=0;waiting=True;service_waiting=True
                        if now>=ex.service_until:
                            if station:
                                station.advance(now)
                                if not station.occupants[n].service_completed:raise RuntimeError('warehouse service incomplete')
                                operation=checkpoint['operation']
                                if operation=='LOADING':
                                    if ex.cargo_units!=0:raise RuntimeError('cargo loaded twice')
                                    ex.cargo_units=1
                                elif operation=='UNLOADING':
                                    if ex.cargo_units!=1:raise RuntimeError('unloading without cargo')
                                    ex.cargo_units=0
                                else:raise RuntimeError('unknown warehouse operation')
                                self.events.append({'time':now,'vehicle_id':n,'event':operation+'_COMPLETE','site':checkpoint['site'],'cargo_units':ex.cargo_units})
                            ex.completed_legs.add(checkpoint['leg'])
                            self.events.append({'time':now,'vehicle_id':n,'event':'TASK_COMPLETE','leg':checkpoint['leg'],'site':checkpoint['site']})
                            ex.checkpoint+=1;ex.service_until=None
                            if 'cycle' in checkpoint and checkpoint['site'].startswith('Q'):
                                self.events.append({'time':now,'vehicle_id':n,'event':'TRANSPORT_CYCLE_RETURNED','cycle':checkpoint['cycle'],'waiting_site':checkpoint['site']})
                            upcoming=next((cp for cp in ex.plan.checkpoints[ex.checkpoint:] if cp['event']=='ARRIVAL'),None)
                            if upcoming and upcoming['leg'] not in ex.started_legs:
                                leg=upcoming['leg'];ex.started_legs.add(leg)
                                self.events.append({'time':now,'vehicle_id':n,'event':'STARTED','leg':leg,'origin':checkpoint['site'],'destination':ex.plan.stops[leg]})
                                if upcoming.get('operation')=='LOADING' and 'cycle' in upcoming:
                                    self.events.append({'time':now,'vehicle_id':n,'event':'TRANSPORT_TASK_RELEASED','cycle':upcoming['cycle'],'source':upcoming['site']})
                    elif remaining<.22 and stopped_at_point(state,ex.plan.point(checkpoint['s']),
                            getattr(ex.plan,'arrival_heading',None) if checkpoint['s']==ex.plan.length else None):
                        if station:
                            if n not in station.occupants:raise RuntimeError('unreserved warehouse arrival')
                            station.arrive(n,now);station.advance(now)
                        service_s=station.config.service_time if station else self.service_s
                        target=0;ex.service_until=now+service_s;waiting=True;service_waiting=True
                        self.events.append({'time':now,'vehicle_id':n,'event':'ARRIVED','leg':checkpoint['leg'],'site':checkpoint['site'],'node':checkpoint['node']})
                        self.events.append({'time':now,'vehicle_id':n,'event':'SERVICING','leg':checkpoint['leg'],'service_s':service_s,'test_station':station is None,'site':checkpoint['site'],'operation':checkpoint.get('operation')})
                elif checkpoint['event']=='GEAR_CHANGE':
                    remaining=max(0,checkpoint['s']-s)
                    target=min(target,math.sqrt(1.2*remaining),remaining*.8)
                    waiting=True
                    if remaining<.22 and abs(state.speed)<.08:
                        ex.progress=max(ex.progress,checkpoint['s'])
                        ex.checkpoint+=1;target=0
                        self.events.append({'time':now,'vehicle_id':n,'event':'GEAR_CHANGE','direction':checkpoint['direction'],'speed_m_s':state.speed})
                elif s>=checkpoint['s']-.2:
                    self.events.append({'time':now,'vehicle_id':n,'event':'TURNAROUND_COMPLETE','leg':checkpoint['leg'],'site':checkpoint['site']})
                    ex.checkpoint+=1
                    if checkpoint['leg']+1<len(ex.plan.stops):
                        leg=checkpoint['leg']+1;ex.started_legs.add(leg)
                        self.events.append({'time':now,'vehicle_id':n,'event':'STARTED','leg':leg,'origin':checkpoint['site'],'destination':ex.plan.stops[leg]})
            record('checkpoint')
            remaining=ex.plan.length-s
            if (remaining<.22 and stopped_at_point(state,ex.plan.point(ex.plan.length),getattr(ex.plan,'arrival_heading',None))
                    and len(ex.completed_legs)==len(ex.plan.stops)
                    and ex.checkpoint==len(ex.plan.checkpoints)):
                if (not self.collect_at_destination and (len(ex.cleared)!=len(ex.plan.visits)
                        or not ex.shared_requested.issubset(ex.shared_cleared))):
                    raise RuntimeError(n+' collected before clearing all resources')
                ex.finished=True;target=0
                self.events.append({'time':now,'vehicle_id':n,'event':'COLLECTED','test_fixture':True})
            if waiting:
                ex.unblocked_since=None
                if abs(state.speed)<.08:
                    if service_waiting:ex.service_wait_s+=dt
                    else:ex.waiting_s+=dt
            elif ex.unblocked_since is None:ex.unblocked_since=now
            if (not ex.finished and not waiting and ex.unblocked_since is not None
                    and now-max(ex.last_progress_time,ex.unblocked_since)>35):
                raise RuntimeError(n+' stalled without a traffic or service wait')
            delta=ex.plan.steering((state.x,state.y),state.heading,s,speed=state.speed,
                                   feedback=ex.steering_feedback)
            commands[n]=(-target if ex.plan.reverse(ex.progress) else target,delta)
        self.maximum_active=max(self.maximum_active,len(self.states))
        self.maximum_moving=max(self.maximum_moving,sum(abs(s.speed)>.1 for s in self.states.values()))
        self.maximum_queue=max(self.maximum_queue,max((len(q) for q in self.scheduler.queues.values()),default=0))
        if getattr(self,'road_guard',None):
            def on_road(proposed):
                return {n:self.road_guard.command(self.states[n],*command,self.ground_heights[n],now)
                        for n,command in proposed.items()}
            for _ in range(3):
                commands=on_road(commands)
                if not self.local_traffic:break
                regulated=self.local_traffic.regulate(self.states,self.executions,commands,self.destination_holds,now,envelope)
                checked=on_road(regulated)
                if checked==regulated:
                    commands=checked;break
                commands=checked
            else:raise RuntimeError('Road and collision command guards did not converge')
        elif self.local_traffic:
            commands=self.local_traffic.regulate(self.states,self.executions,commands,self.destination_holds,now,envelope)
        for n,(target,_) in commands.items():
            self.control_trace.setdefault(n,[]).append(dict(stage='final_local_regulation',target_m_s=target))
        return commands

    def _release_passed_commitments(self):
        """A short road can be crossed between observations; retain its tail,
        but do not leave an entrance booking behind the completed trajectory.
        Measured current occupancy is maintained separately by RoadCapacity.
        """
        remaining={}
        bookings=getattr(self,'local_exit_bookings',{})
        for alias,vehicles in self.capacity.committed.items():
            for name in tuple(vehicles):
                ex=self.executions[name]
                booking=bookings.get((name,alias))
                if booking is not None and (booking[0] is not ex.plan or ex.progress>booking[1]+TAIL_CLEAR_M):
                    self.capacity.release_commitment(name,alias);bookings.pop((name,alias));continue
                if name not in remaining:
                    remaining[name]={self.model.aliases[key] for i,key in enumerate(ex.plan.edges)
                        if ex.plan.edge_ends[i]>=ex.progress-TAIL_CLEAR_M}
                if alias not in remaining[name]:self.capacity.release_commitment(name,alias)
        for key in tuple(bookings):
            if key[0] not in self.capacity.committed[key[1]]:bookings.pop(key)

    def report(self):
        expected=sum(len(p.stops) for p in self.plans.values())
        completed=sum(len(ex.completed_legs) for ex in self.executions.values())
        finished=sum(ex.finished for ex in self.executions.values())
        return {'status':'PASS' if completed==expected and finished==len(self.plans) else 'FAIL',
                'task_count':expected,'completed_tasks':completed,'vehicle_count':len(self.plans),'collected_vehicles':finished,
                'maximum_active_vehicles':self.maximum_active,'maximum_moving_vehicles':self.maximum_moving,
                'maximum_junction_queue':self.maximum_queue,'maximum_parallel_resources':self.scheduler.max_owners,
                'maximum_shared_resources':self.shared_scheduler.max_owners if self.shared_scheduler else 0,
                'maximum_path_error_m':self.maximum_error,'collision_count':self.collision_count,
                'unauthorized_resource_entry_count':self.unauthorized_count,
                'unauthorized_shared_entry_count':self.shared_unauthorized_count,
                'shared_resource_events':self.shared_scheduler.events if self.shared_scheduler else [],'road_capacity_violation_steps':self.capacity_violation_steps,
                'waiting_s':{n:ex.waiting_s for n,ex in self.executions.items()},'simulation_time_s':self.now,
                'cargo_units':{n:ex.cargo_units for n,ex in self.executions.items()},
                'service_occupants':{key:list(station.occupants) for key,station in self.services.items()},
                'maximum_berth_occupants':{key:station.max_reserved for key,station in self.services.items()},
                'test_service_stationary_s':{n:ex.service_wait_s for n,ex in self.executions.items()},
                'controlled_resources_used':sorted({e['intersection_id'] for e in self.scheduler.events if e['event']=='ENTERED'}),
                'road_aliases_used':sorted({self.model.aliases[k] for p in self.plans.values() for k in p.edges if self.model.edges[k]['kind']=='road'}),
                'throughput_tasks_per_hour':completed/max(.001,self.now)*3600,
                'outlet_block_schedule':self.outlet_block_until,
                'task_events':self.events,'traffic_events':self.scheduler.events}
