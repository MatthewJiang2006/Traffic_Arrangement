"""Short-horizon footprint reservations on existing directed trajectories.

Stopped bodies remain obstacles. Reservations predict motion, never teleport
vehicles or release physical occupancy. Ordinary parallel lanes can proceed.
"""
import math
import bisect
from collections import Counter
from functools import lru_cache
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps,separating_gap
from .conflict_timing import ConflictTiming
TAIL_CLEAR_M=6.

@lru_cache(maxsize=512)
def _spatial_boxes(boxes):
    cells={}
    for box in boxes:cells.setdefault((math.floor(box.x/11.),math.floor(box.y/11.)),[]).append(box)
    return cells

def sweeps_intersect(one,two,margin):
    one=tuple(dict.fromkeys(one));two=tuple(dict.fromkeys(two))
    if not one or not two:return False
    if len(one)>len(two):one,two=two,one
    if len(one)<4:
        return any((a.x-b.x)**2+(a.y-b.y)**2<121. and overlaps(a,b,margin) for a in one for b in two)
    cells=_spatial_boxes(two)
    for a in one:
        x,y=math.floor(a.x/11.),math.floor(a.y/11.)
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for b in cells.get((x+dx,y+dy),()):
                    if (a.x-b.x)**2+(a.y-b.y)**2<121. and overlaps(a,b,margin):return True
    return False

class LocalTraffic:
    def __init__(self,*,horizon_s=16.,step_s=.5,margin_m=.35,point_conflicts=False):
        if not (horizon_s>=4 and 0<step_s<=.5 and margin_m>=.2):raise ValueError('Invalid local traffic horizon')
        self.point_conflicts=point_conflicts;self.last_targets={};self.compatible=set()
        self.horizon_s=horizon_s;self.step_s=step_s;self.margin_m=margin_m
        self.wait_since={};self.events=[];self.passages={};self.passage_shapes={};self.passage_denials={};self.paused_by={}
        self.released=[];self.retry_after={};self.now=0.
        self.last_denials={}
        self.maneuver_holds={};self.clearance_shapes={}
        self.yield_holds={};self.yield_since={};self.yield_plans={}
        self.clearance_rights={};self.clearance_pauses={}
        self.resume_priorities={}
        self.compact_permissions=False
        self.proactive_admission=False
        self.braking_advice={};self.braking_advice_signatures={}
        self.timing=ConflictTiming()

    def _forecast(self,name,state,ex,target,hold,envelope,*,brake_after_s=None,allowed_start=None):
        if getattr(self,'clearance_pauses',{}).get(name):target=0.
        dt=self.step_s;s=ex.progress;v=abs(state.speed);plan=ex.plan
        stop=plan.length
        if ex.checkpoint<len(plan.checkpoints):
            cp=plan.checkpoints[ex.checkpoint]
            if cp['event'] in ('ARRIVAL','GEAR_CHANGE'):stop=min(stop,cp['s'])
        if hold is not None:stop=min(stop,hold)
        clearance=getattr(self,'maneuver_holds',{}).get(name)
        if clearance:stop=min(stop,clearance['stop_s'])
        yielding=getattr(self,'yield_holds',{}).get(name)
        if yielding:stop=min(stop,yielding['stop_s'])
        for start,end,entry,exit_s in getattr(self,'action_groups',{}).get(name,()):
            if start==allowed_start or exit_s+TAIL_CLEAR_M<s:continue
            lease=self.passages.get((name,start))
            if lease and lease['plan'] is plan and lease['end']>=exit_s-1e-6:continue
            stop=min(stop,max(0.,entry-7.8))
        boxes=[envelope(state)];aliases=[plan.model.aliases[plan.edges[plan.edge_index(s)]]]
        steps=math.ceil(self.horizon_s/dt)
        for index in range(steps):
            if v<1e-9 and (abs(target)<1e-9 or stop-s<1e-9):
                boxes.extend([boxes[-1]]*(steps-index));aliases.extend([aliases[-1]]*(steps-index));break
            if ex.finished:boxes.append(boxes[0]);aliases.append(aliases[0]);continue
            requested=0. if brake_after_s is not None and index*dt>=brake_after_s else abs(target)
            # Bound next velocity using the distance traveled in this step
            # plus its subsequent stopping distance, rather than today's
            # continuous-time speed bound (which can overshoot by a step).
            available=max(0.,stop-s-.5*v*dt-.3*dt*dt)
            safe_next=max(0.,math.sqrt(1.2*available+(.3*dt)**2)-.3*dt)
            limit=min(requested,plan.speed_cap(s),safe_next)
            next_v=v+max(-.6*dt,min(.6*dt,limit-v))
            # A stop line is a command, not a teleport: preserve actual
            # braking overrun if the measured speed cannot stop before it.
            s=min(plan.length,s+.5*(v+next_v)*dt);v=next_v
            boxes.append(envelope(VehicleState(name,*plan.point(s),plan.heading(s))))
            aliases.append(plan.model.aliases[plan.edges[plan.edge_index(s)]])
        return boxes,aliases

    def prepare(self,states,executions,holds,envelope,now=None):
        if not hasattr(self,"timing"):self.timing=ConflictTiming()
        self.now=0. if now is None else now
        self._executions=executions;self.holds=holds
        self.compatible={pair for pair in self.compatible if all((n,start) in self.passages and id(self.passages[(n,start)]['plan'])==pid for n,pid,start in pair)}
        if not hasattr(self,'released'):self.released=[];self.retry_after={}
        self.passage_denials={}
        if not hasattr(self,'paused_by'):self.paused_by={}
        self.braking={n:self._forecast(n,s,executions[n],0.,holds.get(n),envelope) for n,s in states.items()}
        for key,lease in list(self.passages.items()):
            n=key[0];ex=executions[n]
            if ex.finished or ex.plan is not lease['plan'] or ex.progress>lease['end']+TAIL_CLEAR_M:
                del self.passages[key]
            elif (ex.progress+TAIL_CLEAR_M<ex.plan.edge_starts[key[1]]
                    and abs(states[n].speed)<.08 and self.now-lease.get('created_at',self.now)>=8.):
                self.released.append((key,lease));del self.passages[key]
                self.retry_after[key]=self.now+2.
                self.events.append(dict(event='UNENTERED_PASSAGE_RELEASED',vehicle_id=n,start=key[1],time=self.now))
            elif 'stations' in lease:
                # Each sampled box already includes the entire truck and its
                # tail. Retaining another six metres of old full-body poses
                # locks road space the real tail has already left. Actual
                # bodies/braking paths remain independently protected.
                lease['sweep']=[box for station,box in lease['stations'] if station>=ex.progress-.25]
        self._refresh_yields(states,executions)
        self._protect_maneuver_space(states,executions,envelope)
        self.braking={n:self._forecast(n,s,executions[n],0.,holds.get(n),envelope) for n,s in states.items()}

    def _refresh_yields(self,states,executions):
        """A lease can remain live after its pairwise crossing has cleared."""
        self.yield_holds={}
        for owner,right in list(self.resume_priorities.items()):
            ex=executions[owner];state=states[owner]
            if ex.finished or ex.plan is not right['plan'] or math.dist(right['point'],(state.x,state.y))>=.25:
                del self.resume_priorities[owner]
        for owner,keys in list(self.paused_by.items()):
            ex=executions[owner]
            stations=sorted(((station,box) for key,lease in self.passages.items() if key[0]==owner
                for station,box in lease.get('stations',()) if station>=ex.progress-.25), key=lambda row:row[0])
            owner_sweep=[box for _,box in stations]
            retained=set()
            for key in keys:
                other=key[0]
                if other not in self.braking:continue
                version=self.yield_plans.get((owner,key))
                # A new route must not inherit an old route's precedence.
                lease=self.passages.get(key)
                donor=[b for s,b in lease.get('stations',()) if s>=executions[other].progress-.25] if lease else []
                if lease and version is not None and version is not lease['plan']:donor=[]
                occupied=self.braking[other][0]
                if not sweeps_intersect(owner_sweep,donor+occupied,self.margin_m):continue
                retained.add(key)
                hit=next((s for s,b in stations if sweeps_intersect([b],donor+occupied,self.margin_m)),None)
                if hit is not None:
                    stop=max(0.,hit-.75);previous=self.yield_holds.get(owner)
                    if previous is None or stop<previous['stop_s']:
                        self.yield_holds[owner]=dict(stop_s=stop,blocked_by=other,start=key[1],reason='passage_yield_boundary')
            if self.compact_permissions:
                for other,start in keys-retained:
                    self.events.append(dict(event='PAIRWISE_CONFLICT_CLEARED',vehicle_id=owner,
                        passing_vehicle=other,start=start,time=self.now))
            keys.intersection_update(retained)
            if not keys or not owner_sweep or ex.finished:
                if not keys and owner_sweep and not ex.finished and self.now-self.yield_since.get(owner,self.now)>=8.:
                    self.resume_priorities[owner]=dict(plan=ex.plan,point=(states[owner].x,states[owner].y))
                del self.paused_by[owner]
                self.yield_holds.pop(owner,None);self.yield_since.pop(owner,None)
                self.events.append(dict(event='PASSAGE_YIELD_ENDED',vehicle_id=owner))
        self.yield_plans={k:v for k,v in self.yield_plans.items() if k[1] in self.paused_by.get(k[0],set())}

    def _clearance_stop(self,name,ex):
        """Earliest already-required stop, including unapproved maneuvers."""
        stop=getattr(self,'holds',{}).get(name,ex.plan.length);reason='destination_hold'
        for cp in ex.plan.checkpoints[ex.checkpoint:]:
            if cp['event'] in ('ARRIVAL','GEAR_CHANGE') and cp['s']<stop:
                stop=cp['s'];reason='checkpoint'
        groups=getattr(self,'action_groups',{}).get(name)
        if groups is None:
            from .conflict_timing import maneuver_groups
            groups=maneuver_groups(ex.plan) if hasattr(ex.plan,'edge_starts') else ()
        for start,end,entry,exit_s in groups:
            if exit_s+TAIL_CLEAR_M<ex.progress:continue
            lease=self.passages.get((name,start))
            if lease and lease['plan'] is ex.plan and lease['end']>=exit_s-1e-6:continue
            line=max(0.,entry-7.8)
            if line<stop:stop=line;reason='ungranted_stop_line'
        return stop,reason

    def _clearance_poses(self,name,ex,envelope,states,end):
        """Front-to-back crawl the grantee would drive to satisfy clearance.

        Samples the poses the grantee would occupy while crawling out of the
        owner's space, at a conservative low speed bounded by the same
        acceleration and braking limits as the regulator. An obstacle that
        blocks this motion also blocks the real truck, because the regulator
        rejects exactly this kind of predicted-body overlap.
        """
        dt=self.step_s
        s=float(ex.progress);v=max(0.,abs(states[name].speed))
        poses=[envelope(VehicleState(name,*ex.plan.point(s),ex.plan.heading(s)))]
        for _ in range(math.ceil(self.horizon_s/dt)):
            if s>=end-1e-9:break
            available=max(0.,end-s-.5*v*dt-.3*dt*dt)
            safe_next=max(0.,math.sqrt(1.2*available+(.3*dt)**2)-.3*dt)
            limit=min(.5,ex.plan.speed_cap(s),safe_next)
            next_v=v+max(-.6*dt,min(.6*dt,limit-v))
            s+=.5*(v+next_v)*dt;v=next_v
            poses.append(envelope(VehicleState(name,*ex.plan.point(s),ex.plan.heading(s))))
        return poses

    def _clearance_executable(self,owner,name,ex,states,envelope,end):
        """Can the grantee really clear the paused owner's body?

        The regulator rejects forward motion whenever a predicted body would
        enter the 35 cm margin around another truck. A clearance right that
        ignores that geometry pauses the owner and then cannot be fulfilled,
        which leaves two mutually blocked trucks. Grant the duty only when the
        grantee can actually drive forward out of the owner's space.
        """
        if end-ex.progress<=1e-9:return True
        # The whole span that must be cleared stays on approved forward road;
        # the crawl would otherwise silently stop short of the required end.
        span=max(1,math.ceil((end-ex.progress)/.5))
        for station in (ex.progress+(end-ex.progress)*i/span for i in range(span+1)):
            index=ex.plan.edge_index(station)
            kind=ex.plan.model.edges[ex.plan.edges[index]]['kind']
            if ex.plan.reverse(station):return False
            if kind=='road':continue
            # A forward source exit is also executable when its current action
            # is already approved. Never grant an unapproved access or a turn.
            if kind!='service_access' or not any(
                who==name and lease['plan'] is ex.plan and start<=index
                and ex.plan.edge_starts[start]<=station<=lease['end']
                for (who,start),lease in self.passages.items()):return False
        # The owner stays paused while the grantee clears, so its measured body
        # is the obstacle the grantee has to get past.
        # A third stopped body can wall in the grantee even when the paused
        # owner's body is clear. Granting/retaining that duty would freeze the
        # owner for a truck which cannot fulfill it, closing a waiting cycle.
        occupied=[envelope(state) for other,state in states.items()
                  if other!=name and (other==owner or abs(state.speed)<.15)]
        poses=self._clearance_poses(name,ex,envelope,states,end)
        if len(poses)<2:return False
        return not any(overlaps(pose,body,self.margin_m) for pose in poses[1:] for body in occupied)

    def _grant_clearance(self,owner,start,name,ex,states,conflicts,index,envelope):
        """Keep a forward road truck moving until its complete body clears."""
        if self.proactive_admission:
            from .maneuver_admission import entered_special,prospective_cycle
            owner_ex=self._executions.get(owner)
            if owner_ex and entered_special(owner_ex.plan,owner_ex.progress,start,self.passages[owner,start]['end'],forward_access=True):
                return False
            if prospective_cycle(self,name,{owner},states):return False
        end_index=index
        while end_index+1<len(conflicts) and conflicts[end_index+1]-conflicts[end_index]<=.501:
            end_index+=1
        end=conflicts[end_index]+.5
        if end>ex.plan.length or end-ex.progress>40:return False
        if any(cp['s']<end-.01 for cp in ex.plan.checkpoints[ex.checkpoint:] if cp['event'] in ('ARRIVAL','GEAR_CHANGE')):return False
        limit,reason=self._clearance_stop(name,ex)
        if limit<end:
            signature=(owner,start,name,reason,round(limit,2),round(end,2))
            if not hasattr(self,'clearance_rejections'):self.clearance_rejections=set()
            if signature not in self.clearance_rejections:
                self.events.append(dict(event='CLEARANCE_PRIORITY_REJECTED',vehicle_id=name,
                    yielding_vehicle=owner,time=self.now,reason=reason,stop_s=limit,required_end_s=end))
                self.clearance_rejections.add(signature)
            return False
        if overlaps(envelope(states[name]),envelope(states[owner]),.2):return False
        # A duty to clear is only admissible when the grantee can really leave
        # the owner's space. Pausing a reversing truck for a forward truck that
        # is itself blocked by that body produces reciprocal waiting.
        if not self._clearance_executable(owner,name,ex,states,envelope,end):
            signature=(owner,start,name,'unexecutable_clearance',round(end,2))
            if not hasattr(self,'clearance_rejections'):self.clearance_rejections=set()
            if signature not in self.clearance_rejections:
                self.events.append(dict(event='CLEARANCE_PRIORITY_REJECTED',vehicle_id=name,
                    yielding_vehicle=owner,time=self.now,reason='unexecutable_clearance',required_end_s=end))
                self.clearance_rejections.add(signature)
            return False
        self.clearance_rights[owner,start,name]=dict(plan=ex.plan,owner_plan=self.passages[owner,start]['plan'],end=end)
        self.clearance_pauses.setdefault(owner,set()).add(name)
        self.events.append(dict(event='CLEARANCE_PRIORITY_GRANTED',vehicle_id=name,yielding_vehicle=owner,start=start,end_m=end,time=self.now))
        return True

    def _protect_maneuver_space(self,states,executions,envelope):
        """Keep stopped queues out of a granted access/U-turn sweep.

        Ordinary intersection traffic still uses paired time forecasts. A
        reverse entrance crosses adjacent lanes behind the truck, however;
        stopping other cars at the intersection line can occupy that entrance.
        Use their first swept-body conflict as the upstream holding boundary.
        """
        self.maneuver_holds={}
        self.clearance_pauses={}
        for key,right in list(self.clearance_rights.items()):
            owner,start,name=key;lease=self.passages.get((owner,start));ex=executions.get(name)
            if (lease is None or lease['plan'] is not right['owner_plan'] or ex is None
                    or ex.finished or ex.plan is not right['plan'] or ex.progress>=right['end']):
                del self.clearance_rights[key]
                self.events.append(dict(event='CLEARANCE_PRIORITY_ENDED',vehicle_id=name,yielding_vehicle=owner,time=self.now))
            elif self._clearance_stop(name,ex)[0]<right['end']:
                limit,reason=self._clearance_stop(name,ex)
                del self.clearance_rights[key]
                self.events.append(dict(event='CLEARANCE_PRIORITY_WITHDRAWN',vehicle_id=name,
                    yielding_vehicle=owner,time=self.now,reason=reason,stop_s=limit,required_end_s=right['end'],
                    physical_leases_revoked=False))
            elif self.compact_permissions and not self._clearance_executable(owner,name,ex,states,envelope,right['end']):
                del self.clearance_rights[key]
                self.events.append(dict(event='CLEARANCE_PRIORITY_WITHDRAWN',vehicle_id=name,
                    yielding_vehicle=owner,time=self.now,reason='clearance_no_longer_executable',
                    physical_leases_revoked=False))
            else:self.clearance_pauses.setdefault(owner,set()).add(name)
        if not hasattr(self,'clearance_shapes'):self.clearance_shapes={}
        for (owner,start),lease in self.passages.items():
            plan=lease['plan'];owner_ex=executions[owner]
            canonical=[(station,box) for station,box in lease.get('stations',())
                if station<=lease['end']+1e-7
                and (plan.model.edges[plan.edges[plan.edge_index(station)]]['kind']=='uturn'
                     or (plan.model.edges[plan.edges[plan.edge_index(station)]]['kind']=='service_access'
                         and (plan.reverse(station) or self.proactive_admission)))]
            special=[(station,box) for station,box in canonical if station>=owner_ex.progress-.25]
            if not special:continue
            first_special=canonical[0][0]
            # Cache against canonical geometry, then discard already passed
            # stations at lookup. Progress must not create a new full-route scan.
            all_special=tuple(box for _,box in canonical)
            for name,ex in executions.items():
                if name==owner or name not in states or ex.finished:continue
                if (owner,start,name) in self.clearance_rights:continue
                # A granted yield is a single precedence decision: the
                # yielding owner cannot impose a future clearance hold on
                # the very truck it has already allowed to pass.
                if any(grant[0]==name for grant in self.paused_by.get(owner,set())):continue
                # The leader may lie beyond the owner's planned gear stop,
                # so projecting onto that shorter future route misses it.
                # Also observe physical order on the current directed lane.
                dx,dy=states[name].x-states[owner].x,states[name].y-states[owner].y
                yaw=states[owner].heading;co,si=math.cos(yaw),math.sin(yaw)
                forward=dx*co+dy*si;lateral=abs(-dx*si+dy*co)
                yaw_difference=abs((states[name].heading-yaw+math.pi)%(2*math.pi)-math.pi)
                if .1<forward<40. and lateral<1.4 and yaw_difference<math.pi/3:continue
                # A following truck cannot reserve the unique approach in
                # front of its actual lane leader and make that leader wait
                # for its future maneuver. The leader must clear first.
                error,station=plan.project((states[name].x,states[name].y),owner_ex.progress,behind=0,ahead=40.)
                angle=abs((states[name].heading-plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
                if error<1.4 and angle<math.pi/3 and owner_ex.progress+.1<station<owner_ex.progress+40.:continue
                # Two special actions must use the same precedence at entry
                # and throughout execution. An entered later reverse cannot
                # retroactively stop the earlier committed reverse. This only
                # removes its future-space hold, never observed body checks.
                special_older=not any(grant[0]==owner for grant in self.paused_by.get(name,set())) and any(who==name and own['plan'] is ex.plan
                    and ex.progress>=ex.plan.edge_starts[own_start]-TAIL_CLEAR_M
                    and (own.get('created_at',self.now),name)<(lease.get('created_at',self.now),owner)
                    and sweeps_intersect(self._special_boxes(own,ex.progress-.25),[b for _,b in special],self.margin_m)
                    for (who,own_start),own in self.passages.items())
                if special_older:
                    signature=(owner,start,name,id(plan),id(ex.plan))
                    if not hasattr(self,'special_precedence_events'):self.special_precedence_events=set()
                    if signature not in self.special_precedence_events:
                        self.events.append(dict(event='SPECIAL_MANEUVER_PRECEDENCE',vehicle_id=name,
                            yielding_vehicle=owner,time=self.now,reason='earlier_committed_special_action',
                            physical_checks_retained=True))
                        self.special_precedence_events.add(signature)
                    continue
                older=False
                if owner_ex.progress<first_special+.25:
                    # A later future reverse action must not stop a truck
                    # already committed to an older passage. Finish that
                    # passage first; measured bodies remain guarded by SAT.
                    older=any(who==name and own['plan'] is ex.plan
                        and ex.progress>=ex.plan.edge_starts[own_start]-TAIL_CLEAR_M
                        and (own.get('created_at',self.now),name)<(lease.get('created_at',self.now),owner)
                        for (who,own_start),own in self.passages.items())
                key=(id(ex.plan),id(plan),start)
                if key not in self.clearance_shapes:
                    if len(self.clearance_shapes)>=512:self.clearance_shapes.pop(next(iter(self.clearance_shapes)))
                    cells=_spatial_boxes(all_special);conflicts=[]
                    for i in range(math.ceil(ex.plan.length/.5)+1):
                        station=min(ex.plan.length,i*.5)
                        body=envelope(VehicleState(name,*ex.plan.point(station),ex.plan.heading(station)))
                        x,y=math.floor(body.x/11.),math.floor(body.y/11.)
                        if any(overlaps(body,b,self.margin_m) for dx in (-1,0,1) for dy in (-1,0,1)
                               for b in cells.get((x+dx,y+dy),())):conflicts.append(station)
                    # Retain the plans too, so an object id cannot be reused
                    # for different geometry while its cache entry is alive.
                    self.clearance_shapes[key]=(ex.plan,plan,conflicts)
                conflicts=self.clearance_shapes[key][2];i=bisect.bisect_left(conflicts,ex.progress-.25)
                # Only the still-needed portion of the maneuver is protected.
                # An obsolete first conflict must not hide a later, still
                # active one; otherwise the holding line appears only after
                # the approaching truck has already entered that later space.
                limit=ex.progress+max(35.,abs(states[name].speed)**2/1.2+12.)
                needed=[b for _,b in special]
                while i<len(conflicts) and conflicts[i]<=limit:
                    station=conflicts[i]
                    body=envelope(VehicleState(name,*ex.plan.point(station),ex.plan.heading(station)))
                    if sweeps_intersect([body],needed,self.margin_m):break
                    i+=1
                else:continue
                stop=max(0.,station-(1.5 if self.proactive_admission else .75))
                if older and self.proactive_admission:
                    # Older permission protects completion of its approved
                    # section, not entry into the next dock's clearance space.
                    # Hold downstream of that section before the dock starts.
                    older=any(who==name and own['plan'] is ex.plan
                        and (own.get('created_at',self.now),name)<(lease.get('created_at',self.now),owner)
                        and ex.progress<=own.get('outlet',own['end']+TAIL_CLEAR_M)
                        and stop<own.get('outlet',own['end']+TAIL_CLEAR_M)
                        for (who,_),own in self.passages.items())
                # Preserve the already committed exit even after its old
                # junction permission ends. Also recover a road truck already
                # inside a hold: freezing it there cannot clear the reverse.
                from logistics_park.runtime.traffic_policy import stop_line_speed
                settled=abs(states[name].speed)<.15 and stop_line_speed(max(0.,stop-ex.progress))==0.
                if (older or stop<=ex.progress+.25 or settled) and self._grant_clearance(owner,start,name,ex,states,conflicts,i,envelope):continue
                if older:continue
                previous=self.maneuver_holds.get(name)
                if previous is None or stop<previous['stop_s']:
                    self.maneuver_holds[name]=dict(stop_s=stop,blocked_by=owner,start=start,reason='reserved_maneuver_clearance')

    def _special_boxes(self,lease,from_s=0.):
        plan=lease['plan']
        return [box for station,box in lease.get('stations',()) if from_s<=station<=lease['end']+1e-7
            and (plan.model.edges[plan.edges[plan.edge_index(station)]]['kind']=='uturn'
                 or (plan.model.edges[plan.edges[plan.edge_index(station)]]['kind']=='service_access'
                     and (plan.reverse(station) or self.proactive_admission)))]

    def _time_compatible(self,name,other,ex,start,states,envelope):
        if not self.point_conflicts or not hasattr(self,'_executions'):return False
        # A forecast never replaces observed occupancy: stopped/blocked owners
        # are predicted braking to rest, and actual paired-body checks remain.
        owner=self._executions[other]
        if owner.finished or self.paused_by.get(other):return False
        # Full reverse/U-turn clearance protection is spatial. A temporal
        # overlap exception here would grant two permissions whose downstream
        # protection then stops both cars. Retain the existing single-owner
        # admission/yield checks for intersecting special sweeps; ordinary
        # junctions and disjoint special actions still use time windows.
        group=next((g for g in getattr(self,'action_groups',{}).get(name,()) if g[0]==start),None)
        if group:
            samples=self.timing.sweep(ex.plan,group,envelope)[1]
            own_special=self._special_boxes(dict(plan=ex.plan,end=group[3],stations=samples),ex.progress-.25)
            if self.proactive_admission and own_special:return False
            if own_special and any(who==other and sweeps_intersect(own_special,
                    self._special_boxes(lease,owner.progress-.25),self.margin_m)
                    for (who,_),lease in self.passages.items()):return False
        if self.proactive_admission and any(who==other and self._special_boxes(lease,owner.progress-.25)
                for (who,_),lease in self.passages.items()):return False
        other_target=self.last_targets.get(other,abs(states[other].speed))
        if other_target<.15:return False
        error,station=ex.plan.project((states[other].x,states[other].y),ex.progress,behind=0,ahead=40.)
        angle=abs((states[other].heading-ex.plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
        following=error<1.4 and angle<math.pi/3 and station>ex.progress+.1
        target=ex.plan.speed_cap(ex.progress)
        if following:
            from logistics_park.runtime.traffic_policy import needs_maneuver_room
            if needs_maneuver_room(owner):return False
            # Claim a following passage at the leader's speed. Predicting a
            # full-speed catch-up here would deny the permission even though
            # the downstream controller safely follows the leader.
            target=min(target,other_target)
        own=self._forecast(name,states[name],ex,target,self.holds.get(name),envelope,allowed_start=start)[0]
        theirs=self._forecast(other,states[other],owner,other_target,self.holds.get(other),envelope)[0]
        if any(overlaps(a,b,self.margin_m+.15) for a,b in zip(own,theirs)):return False
        # Outside the forecast horizon there must still be a finite, disjoint
        # conflict window; do not grant unknown future crossings optimistically.
        own_group=next((g for g in self.action_groups.get(name,()) if g[0]==start),None)
        other_group=next((g for g in self.action_groups.get(other,()) if g[3]+6>=owner.progress),None)
        if own_group and other_group:
            conflict=self.timing.conflict(self.timing.sweep(ex.plan,own_group,envelope),self.timing.sweep(owner.plan,other_group,envelope))
            if conflict:
                from .conflict_timing import arrival_profile
                a,b,c,d=conflict
                if d+.5>=owner.progress and b+.5>=ex.progress:
                    # Same-lane following uses the paired braking/body forecast,
                    # not the entire shared-lane time interval.
                    if not following:
                        p=arrival_profile(ex,states[name],self.now);q=arrival_profile(owner,states[other],self.now,target_speed=other_target)
                        if not (p.eta(b+.5)+1.<=min(self.horizon_s,q.eta(c)) or q.eta(d+.5)+1.<=min(self.horizon_s,p.eta(a))):return False
        return True

    def passage_outlet(self,ex,start,end):
        ordinary=self.compact_permissions and not any(
            ex.plan.model.edges[k].get('reverse') or ex.plan.model.edges[k]['kind']=='uturn'
            for k in ex.plan.edges[start:end+1])
        return ordinary,min(ex.plan.length,ex.plan.edge_ends[end]+(6. if ordinary else 11.344))

    def claim_passage(self,name,ex,start,end,states,boxes,envelope):
        # Reserve the whole maneuver and enough road for its complete exit.
        # Parallel paths that do not intersect keep separate permissions.
        existing=self.passages.get((name,start))
        if existing is not None and existing['end']>=ex.plan.edge_ends[end]-1e-6:return True
        if getattr(self,'retry_after',{}).get((name,start),0.)>getattr(self,'now',0.):return False
        if self.paused_by.get(name) or self.clearance_pauses.get(name):return False
        # Include the approach: a permission cannot be fulfilled if its
        # yielding owner is the physical leader between this truck and entry.
        plan=ex.plan;entry=max(0.,math.floor(ex.progress*4)/4);exit_s=plan.edge_ends[end]
        if self.proactive_admission:
            self.events.append(dict(event='ADMISSION_PROPOSED',vehicle_id=name,start=start,
                time=self.now,entry_s=plan.edge_starts[start],exit_s=exit_s))
        ordinary,outlet=self.passage_outlet(ex,start,end)
        if ordinary:
            # A normal right-of-way starts at its nose-safe holding line,
            # rather than owning the entire upstream road. Each sweep pose
            # includes the full truck; six rear-axle metres of outlet already
            # cover the body plus compact following headroom.
            entry=max(entry,plan.edge_starts[start]-7.8)
        key=(plan.edges,start,end,entry)
        if key not in self.passage_shapes:
            if len(self.passage_shapes)>=256:self.passage_shapes.pop(next(iter(self.passage_shapes)))
            # Curved truck bodies can intersect between 1.5 m samples even
            # when every sampled pair is clear. Use fine canonical stations
            # for the same footprint checks used during motion prediction.
            count=max(1,math.ceil((outlet-entry)/.25))
            self.passage_shapes[key]=[envelope(VehicleState(name,*plan.point(entry+(outlet-entry)*i/count),plan.heading(entry+(outlet-entry)*i/count))) for i in range(count+1)]
        sweep=self.passage_shapes[key]
        def deny(other,reason):
            self.passage_denials.setdefault(name,[]).append(dict(blocked_by=other,reason=reason,start=start))
            if self.proactive_admission:
                self.events.append(dict(event='ADMISSION_REJECTED',vehicle_id=name,
                    blocked_by=other,reason=reason,start=start,time=self.now))
            return False
        def intersects(one,two):
            return sweeps_intersect(one,two,self.margin_m)
        prospective_stations=[(entry+(outlet-entry)*i/(len(sweep)-1),box) for i,box in enumerate(sweep)]
        special_sweep=self._special_boxes(dict(plan=plan,end=exit_s,stations=prospective_stations)) if self.proactive_admission else []
        yielding=set();withdraw=[];compatible=set()
        for key,lease in self.passages.items():
            owner=key[0]
            if owner!=name and intersects(sweep,lease['sweep']):
                if self.proactive_admission:
                    from .maneuver_admission import entered_special
                    active=getattr(self,'_executions',{}).get(owner)
                    if active and entered_special(lease['plan'],active.progress,key[1],lease['end'],forward_access=True):
                        return deny(owner,'entered_special_conflict')
                if self._time_compatible(name,owner,ex,start,states,envelope):
                    compatible.add((owner,key[1]));continue
                # Keep the owner's permission and physical occupancy, but
                # pause it while a complete, clear passage goes past its
                # stopped body. Never brake a moving owner for this purpose.
                other_ex=self._executions[owner] if hasattr(self,'_executions') else None
                future=bool(other_ex and other_ex.progress+TAIL_CLEAR_M<other_ex.plan.edge_starts[key[1]])
                # A clear crossing sweep alone is insufficient: revoking the
                # permission reinstates a stop line before the action. Keep
                # the permission once the owner cannot brake before that line.
                if future:
                    speed=abs(states[owner].speed)
                    available=other_ex.plan.edge_starts[key[1]]-7.8-other_ex.progress
                    stopping=speed*speed/1.2+speed*self.step_s+.3*self.step_s**2+.5
                    if available<stopping:return deny(owner,'passage_stop_line_committed')
                own_window=self.timing.windows.get(name,{})
                owner_window=self.timing.windows.get(owner,{})
                earlier=(own_window.get('entry_eta_s',math.inf)+1.<owner_window.get('entry_eta_s',0.))
                if abs(states[owner].speed)>.08 and not (future and earlier):return deny(owner,'passage_reservation')
                if intersects(sweep,boxes[owner]) or intersects(sweep,self.braking[owner][0]):return deny(owner,'passage_occupied_body')
                prior=self.paused_by.get(owner,set())
                clears_grantee=False
                pending={grantee for grantee,_ in prior};visited=set()
                while pending:
                    grantee=pending.pop()
                    if grantee in visited:continue
                    visited.add(grantee)
                    leader_ex=getattr(self,'_executions',{}).get(grantee)
                    if grantee not in states or leader_ex is None:continue
                    # Blocking an already passing truck's lane leader would
                    # prevent that truck from clearing the yielding owner.
                    for leader,pose in states.items():
                        if leader in visited or math.dist((pose.x,pose.y),(states[grantee].x,states[grantee].y))>40.:continue
                        error,station=leader_ex.plan.project((pose.x,pose.y),leader_ex.progress,behind=0,ahead=40.)
                        angle=abs((pose.heading-leader_ex.plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
                        if error<1.4 and angle<math.pi/3 and leader_ex.progress+.1<station<leader_ex.progress+40.:
                            if leader==name:clears_grantee=True;break
                            pending.add(leader)
                    if clears_grantee:break
                aged=(prior and self.now-self.yield_since.get(owner,self.now)>=8.) or owner in self.resume_priorities
                if (aged and name not in {k[0] for k in prior} and not clears_grantee):
                    return deny(owner,'aged_yield_priority')
                if other_ex and other_ex.progress+TAIL_CLEAR_M<other_ex.plan.edge_starts[key[1]]:
                    withdraw.append((key,lease))
                else:yielding.add(owner)
        for other,physical in boxes.items():
            if other==name:continue
            if any(owner==other for owner,_ in compatible):continue
            if intersects(sweep,physical):return deny(other,'passage_occupied_body')
            # An approaching vehicle must have room to brake outside a newly
            # reserved maneuver, including those currently at road speed.
            if intersects(sweep,self.braking[other][0]):return deny(other,'passage_braking_path')
            if special_sweep and abs(states[other].speed)>.08:
                # Admission and the protected holding line need the same
                # stopping room. A merely non-overlapping nominal brake pose
                # leaves no allowance for sampled control/pose correction.
                # Require 1.5 longitudinal metres beyond the full brake pose
                # before starting access/reverse/U-turn protection.
                other_ex=self._executions[other]
                last=self.braking[other][0][-1]
                rear=(last.x-.75*math.cos(last.heading),last.y-.75*math.sin(last.heading))
                _,station=other_ex.plan.project(rear,other_ex.progress,behind=.5,
                    ahead=max(35.,abs(states[other].speed)**2/1.2+6.))
                probes=[envelope(VehicleState(other,*other_ex.plan.point(min(other_ex.plan.length,station+.25*i)),
                                             other_ex.plan.heading(min(other_ex.plan.length,station+.25*i)))) for i in range(7)]
                if intersects(special_sweep,probes):return deny(other,'special_hold_stopping_room')
        if self.proactive_admission:
            from .maneuver_admission import prospective_cycle
            cycle=prospective_cycle(self,name,yielding,states,resolved_passage_start=start)
            if cycle:
                self.events.append(dict(event='ADMISSION_WAIT_CYCLE_PREVENTED',vehicle_id=name,
                    vehicles=cycle,start=start,time=self.now))
                return deny(next(n for n in cycle if n!=name),'prospective_wait_cycle')
        stations=prospective_stations
        for old_key,lease in withdraw:
            self.released.append((old_key,lease));del self.passages[old_key]
            self.retry_after[old_key]=self.now+2.
            self.events.append(dict(event='UNENTERED_PASSAGE_RELEASED',vehicle_id=old_key[0],start=old_key[1],time=self.now))
        self.compatible.update(frozenset(((name,id(plan),start),(other,id(self._executions[other].plan),other_start))) for other,other_start in compatible)
        self.passages[(name,start)]=dict(plan=plan,start=start,end=exit_s,outlet=outlet,sweep=sweep,stations=stations,created_at=getattr(self,'now',0.))
        self.events.append(dict(event='PASSAGE_GRANTED',vehicle_id=name,start=start,time=self.now,distance_to_entry_m=plan.edge_starts[start]-ex.progress))
        if self.proactive_admission:
            self.events.append(dict(event='ADMISSION_COMMITTED',vehicle_id=name,start=start,
                time=self.now,paused_vehicles=sorted(yielding),approved_end_s=outlet))
        for owner in yielding:
            self.paused_by.setdefault(owner,set()).add((name,start))
            self.yield_since.setdefault(owner,self.now)
            self.yield_plans[owner,(name,start)]=plan
            self.wait_since.setdefault(owner,0.)
            self.events.append(dict(event='PASSAGE_YIELD_STARTED',vehicle_id=owner,passing_vehicle=name,start=start))
        return True

    def regulate(self,states,executions,commands,holds,now,envelope):
        from logistics_park.runtime.traffic_policy import stop_line_speed
        commands=dict(commands)
        for n,(target,steer) in list(commands.items()):
            if self.clearance_pauses.get(n) or (self.paused_by.get(n) and (not self.point_conflicts or n not in self.yield_holds)):
                commands[n]=(0.,steer)
            elif n in self.yield_holds:
                cap=stop_line_speed(max(0.,self.yield_holds[n]['stop_s']-executions[n].progress))
                commands[n]=(math.copysign(min(abs(target),cap),target),steer)
        forecasts=dict(self.braking) if hasattr(self,'braking') else {n:self._forecast(n,s,executions[n],0.,holds.get(n),envelope) for n,s in states.items()}
        remaining={n for n in states if not executions[n].finished};ordered=[]
        ahead_by={n:set() for n in remaining}
        # Compute physical lane order once, rather than projecting every pair
        # again for each item removed from the ordering queue.
        for n in remaining:
            ex=executions[n]
            for other in remaining-{n}:
                state=states[other]
                if math.dist((states[n].x,states[n].y),(state.x,state.y))>30:continue
                error,s=ex.plan.project((state.x,state.y),ex.progress,behind=0,ahead=30)
                angle=abs((state.heading-ex.plan.heading(s)+math.pi)%(2*math.pi)-math.pi)
                if error<1.4 and angle<math.pi/3 and ex.progress+.1<s<ex.progress+30:ahead_by[n].add(other)
        while remaining:
            leaders=[n for n in remaining if not ahead_by[n]&remaining]
            rank={n:i for i,n in enumerate(self.timing.order)}
            def commitment(n):
                # A truck already inside a live maneuver must finish it to
                # clear the lane/junction. When two stopped trucks block each
                # other, running the committed one first breaks the tie along
                # the direction its own permission already requires, instead
                # of letting the order decide against it.
                ex=executions[n]
                return any(key[0]==n and ex.progress>=ex.plan.edge_starts[key[1]]-1e-6
                           for key in self.passages)
            chosen=min(leaders or list(remaining),key=lambda n:(not commitment(n),rank.get(n,math.inf),-abs(states[n].speed),self.wait_since.get(n,now),n))
            ordered.append(chosen);remaining.remove(chosen)
        counts=[Counter(f[1][i] for f in forecasts.values()) for i in range(len(next(iter(forecasts.values()))[1]))]
        self.last_denials={};self.braking_advice={}
        result=dict(commands)
        for n in ordered:
            requested,steer=commands[n];best=0.;selected=forecasts[n]
            if abs(requested)<1e-9:
                # The output is already a brake command. Running every SAT
                # trial cannot make it safer; actual body audits still run.
                continue
            for i,alias in enumerate(forecasts[n][1]):counts[i][alias]-=1
            recovery={other for other,(occupied,_) in forecasts.items() if other!=n
                and abs(states[n].speed)<.08 and abs(states[other].speed)<.08
                and overlaps(forecasts[n][0][0],occupied[0],self.margin_m)
                and not overlaps(forecasts[n][0][0],occupied[0],.2)}
            maximum=min(abs(requested),.5) if recovery else abs(requested)
            crawl_floor=0. if recovery else min(.8,abs(requested)*.5)
            if recovery:
                self.events.append(dict(event='CLEARANCE_RECOVERY_CHECK',vehicle_id=n,time=now,
                    blocked_by=sorted(recovery),maximum_m_s=maximum,crawl_filter_bypassed=True))
            candidates=sorted({maximum*f for f in (1.,.75,.5,.25,.1,.05,.02,.01,0.)},reverse=True)
            # Full-speed itineraries first. If none is safe, try a finite
            # command interval followed by complete braking. The horizon and
            # all paired-body/capacity tests stay unchanged; only the assumed
            # future control policy changes. The next tick must revalidate.
            trials=[(target,None) for target in candidates if target>0]
            if self.point_conflicts:
                trials.extend((target,interval) for interval in (4.,3.,2.5)
                              for target in candidates if target>0 and target>=crawl_floor)
            trials.append((0.,None))
            selected_interval=None
            for target,interval in trials:
                if self.point_conflicts and 0<target<crawl_floor and not ahead_by[n]:
                    self.last_denials[n]=dict(reason='anti_crawl_filter',maximum_m_s=maximum,minimum_m_s=crawl_floor)
                    continue
                candidate=self._forecast(n,states[n],executions[n],target,holds.get(n),envelope,brake_after_s=interval)
                safe=True
                for other,(occupied,_) in forecasts.items():
                    if other==n:continue
                    previous_gap=None
                    for sample,(a,b) in enumerate(zip(candidate[0],occupied)):
                        if other in recovery:
                            gap=separating_gap(a,b,self.margin_m)
                            # Monotonic escape applies while restoring a
                            # violated soft margin. Once separated at the full
                            # margin, ordinary safe motion may reduce a still
                            # positive gap; it must never re-enter the margin.
                            worsens=(previous_gap is not None and gap<=0. and
                                     (previous_gap>0. or gap<previous_gap-1e-6))
                            if overlaps(a,b,.2) or worsens:
                                self.last_denials[n]=dict(blocked_by=other,reason='clearance_not_increasing');safe=False;break
                            previous_gap=gap;continue
                        if math.hypot(a.x-b.x,a.y-b.y)<11. and overlaps(a,b,self.margin_m):
                            self.last_denials[n]=dict(blocked_by=other,reason='predicted_body',prediction_time_s=sample*self.step_s,trial_speed_m_s=target,brake_after_s=interval);safe=False;break
                    if not safe:break
                if safe:
                    for key,lease in self.passages.items():
                        owner=key[0]
                        if owner==n:continue
                        # Admission still protects entry to the maneuver. On
                        # ordinary approaches, observed paired trajectories
                        # above replace an owner's entire future spatial sweep.
                        if self.point_conflicts:continue
                        if self.point_conflicts and any(frozenset(((n,id(executions[n].plan),own_start),(owner,id(lease['plan']),key[1]))) in self.compatible for who,own_start in self.passages if who==n):continue
                        if any(grant[0]==n for grant in self.paused_by.get(owner,set())):continue
                        # Protect immediate motion plus the full physical
                        # braking path. A distant future crossing is governed
                        # by its stop line and must not freeze this road now.
                        emergency=self._forecast(n,states[n],executions[n],target,holds.get(n),envelope,brake_after_s=self.step_s)
                        if sweeps_intersect(emergency[0][1:],lease['sweep'],self.margin_m):
                            self.last_denials[n]=dict(blocked_by=owner,reason='reserved_maneuver');safe=False;break
                if safe:
                    capacities=executions[n].plan.model.raw['physical_capacities']
                    for index,alias in enumerate(candidate[1]):
                        count=1+counts[index][alias]+sum(other!=n and forecasts[other][1][index]!=alias for other in getattr(self,'committed',{}).get(alias,()))
                        if count>capacities.get(alias,1):self.last_denials[n]=dict(resource=alias,reason='capacity');safe=False;break
                if safe:
                    best=target;selected=candidate;selected_interval=interval
                    if target>0:self.last_denials.pop(n,None)
                    break
            if selected_interval is not None:
                advice=dict(brake_after_s=selected_interval,target_m_s=best,
                            command_revalidation_s=2.)
                self.braking_advice[n]=advice
                signature=(round(best,1),selected_interval)
                if self.braking_advice_signatures.get(n)!=signature:
                    self.events.append(dict(event='SAFE_BRAKING_PREFIX',vehicle_id=n,time=now,**advice))
                self.braking_advice_signatures[n]=signature
            else:self.braking_advice_signatures.pop(n,None)
            forecasts[n]=selected
            for i,alias in enumerate(selected[1]):counts[i][alias]+=1
            result[n]=(math.copysign(best,requested),steer)
            if best+.01<abs(requested):
                self.wait_since.setdefault(n,now)
            else:self.wait_since.pop(n,None)
        self.last_targets={n:abs(v[0]) for n,v in result.items()}
        return result
