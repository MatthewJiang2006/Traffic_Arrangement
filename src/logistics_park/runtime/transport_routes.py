"""Legal itineraries and efficient geometry shared by simulated/UE execution."""
import bisect
import json
import math
from dataclasses import dataclass
from pathlib import Path
from logistics_park.map.map_loader import load_map
from logistics_park.map.geometry import Polyline
from logistics_park.routing.shortest_path import shortest_path,NoRouteError
from logistics_park.runtime.speed_limits import SpeedLimits,edge_speed_limit


@dataclass
class Visit:
    resource: str
    start: float
    end: float
    incoming: str
    connector: str
    outgoing: str
    first_index: int
    last_index: int


@dataclass
class SharedVisit:
    channel_id: str
    start: float
    end: float
    first_index: int
    last_index: int


class TransportMap:
    def __init__(self,path):
        self.raw=json.loads(Path(path).read_text(encoding='utf-8'))
        self.graph=load_map(path)
        self.edges={e['edge_id']:e for e in self.raw['edges']}
        self.sites=self.raw['sites'];self.zones=self.raw['control_zones']
        self.shared_resources=self.raw.get('shared_resources', {})
        self.aliases={k:e['capacity_alias'] for k,e in self.edges.items()}
        self.physical_paths={k:Polyline(tuple(map(tuple,p))) for k,p in self.raw['physical_centerlines'].items()}

    def plan(self,origin,stops,algorithm='astar',blocked_edges=frozenset()):
        if origin not in self.sites or not stops or any(s not in self.sites for s in stops):
            raise ValueError('unknown or empty transport itinerary')
        edges=[];checkpoints=[];current=self.sites[origin]['start']['node_id'];incoming=None
        for leg,site in enumerate(stops):
            if site==(origin if leg==0 else stops[leg-1]):raise ValueError('task needs distinct sites')
            candidates=[]
            for finish in self.sites[site]['finish_nodes']:
                try:
                    route=shortest_path(self.graph,current,finish,algorithm,incoming,blocked_edges)
                    turn=shortest_path(self.graph,finish,self.sites[site]['start']['node_id'],algorithm,route.edges[-1],blocked_edges)
                    candidates.append((route.travel_time+turn.travel_time,finish,route,turn))
                except NoRouteError:continue
            if not candidates:raise NoRouteError('no transport arrival/turnaround for '+site)
            _,finish,route,turn=min(candidates,key=lambda c:(c[0],c[1]))
            edges.extend(route.edges)
            checkpoints.append({'leg':leg,'site':site,'node':finish,'after_edge_count':len(edges),'event':'ARRIVAL'})
            edges.extend(turn.edges)
            checkpoints.append({'leg':leg,'site':site,'node':turn.destination,'after_edge_count':len(edges),'event':'TURNAROUND_COMPLETE'})
            incoming=edges[-1];current=turn.destination
        tail=shortest_path(self.graph,current,self.sites[stops[-1]]['egress_node'],algorithm,incoming,blocked_edges)
        edges.extend(tail.edges)
        self.graph.validate_route(tuple(edges))
        return RouteGeometry(self,edges,checkpoints,origin,list(stops))


class RouteGeometry:
    def __init__(self,model,edges,checkpoints,origin,stops):
        self.model=model;self.edges=tuple(edges);self.origin=origin;self.stops=stops
        self.points=[];self.stations=[];self.edge_starts=[];self.edge_ends=[];self.point_edges=[]
        s=0.
        for index,key in enumerate(edges):
            self.edge_starts.append(s)
            path=model.edges[key]['centerline']
            if not self.points:self.points.append(tuple(path[0]));self.stations.append(0.);self.point_edges.append(index)
            for a,b in zip(path,path[1:]):
                distance=math.dist(a,b);count=max(1,math.ceil(distance/.25))
                for j in range(1,count+1):
                    t=j/count;point=(a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t)
                    s+=math.dist(self.points[-1],point)
                    if math.dist(self.points[-1],point)<1e-8:continue
                    self.points.append(point);self.stations.append(s);self.point_edges.append(index)
            self.edge_ends.append(s)
        self.length=s
        self.checkpoints=[{**c,'s':self.edge_ends[c['after_edge_count']-1]} for c in checkpoints]
        self.visits=[]
        for i,key in enumerate(edges):
            resource=model.edges[key].get('resource')
            if not resource:continue
            previous=next((v for v in reversed(self.visits) if v.resource==resource),None)
            if previous and self.edge_starts[i]-previous.end<8.5:
                previous.end=self.edge_ends[i];previous.last_index=i
            else:
                alias=model.aliases[key]
                self.visits.append(Visit(resource,self.edge_starts[i],self.edge_ends[i],
                                         model.aliases[edges[max(0,i-1)]],alias,alias,i,i))
        for visit in self.visits:
            visit.outgoing=model.aliases[edges[min(len(edges)-1,visit.last_index+1)]]
        self.visits.sort(key=lambda v:v.start)
        self.shared_visits=[]
        for channel_id, spec in model.shared_resources.items():
            edge_ids=set(spec.get('edge_ids', ()))
            for index in [i for i, key in enumerate(edges) if key in edge_ids]:
                if (self.shared_visits and self.shared_visits[-1].channel_id == channel_id
                        and not model.edges[edges[index]].get('shared_visit_start')
                        and self.edge_starts[index]-self.shared_visits[-1].end < 8.5):
                    visit=self.shared_visits[-1]
                    visit.end=self.edge_ends[index];visit.last_index=index
                else:
                    self.shared_visits.append(SharedVisit(channel_id,self.edge_starts[index],
                                                          self.edge_ends[index],index,index))
        limits=SpeedLimits(**model.raw.get('speed_limits',{}));caps=[]
        for index in self.point_edges:
            e=model.edges[edges[index]]
            speed=limits.uturn_m_s if e['kind']=='uturn' else limits.turn_m_s if e['kind']=='lane_change' or e.get('maneuver') in ('LEFT','RIGHT') else limits.straight_m_s
            caps.append(min(speed,edge_speed_limit(e,model,limits)))
        caps[-1]=0.
        for checkpoint in self.checkpoints:
            if checkpoint['event']=='GEAR_CHANGE':
                i=min(len(caps)-1,bisect.bisect_left(self.stations,checkpoint['s']))
                caps[i]=0.
        for i in range(len(caps)-2,-1,-1):
            caps[i]=min(caps[i],math.sqrt(caps[i+1]**2+2*limits.approach_deceleration_m_s2*(self.stations[i+1]-self.stations[i])))
        self.speed_caps=caps

    def edge_index(self,s):return min(len(self.edges)-1,bisect.bisect_right(self.edge_ends,s+1e-7))

    def point(self,s):
        s=max(0,min(self.length,s));i=min(len(self.points)-2,max(0,bisect.bisect_right(self.stations,s)-1))
        a,b=self.points[i:i+2];t=(s-self.stations[i])/max(1e-9,self.stations[i+1]-self.stations[i])
        return (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))

    def heading(self,s):
        if not hasattr(self,'has_reverse'):
            self.has_reverse=any(self.model.edges[k].get('reverse') for k in self.edges)
        if self.has_reverse:
            index=self.edge_index(s);start,end=self.edge_starts[index],self.edge_ends[index]
            if end-start<.05:
                # Sub-centimetre attachment seams encode endpoint snapping,
                # not an instantaneous change in a truck's orientation.
                # Inherit a substantial adjacent segment in the same gear.
                for neighbors in (range(index-1,-1,-1),range(index+1,len(self.edges))):
                    for j in neighbors:
                        if bool(self.model.edges[self.edges[j]].get('reverse'))!=self.reverse(s):break
                        if self.edge_ends[j]-self.edge_starts[j]>=.05:
                            station=self.edge_ends[j]-.02 if j<index else self.edge_starts[j]+.02
                            return self.heading(station)
            a=self.point(max(start,min(s-.02,end-.04)))
            b=self.point(min(end,max(s+.02,start+.04)))
            return math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if self.reverse(s) else 0)
        a=self.point(max(0,s-.02));b=self.point(min(self.length,s+.02))
        return math.atan2(b[1]-a[1],b[0]-a[0])

    def reverse(self,s):
        return bool(self.model.edges[self.edges[self.edge_index(s)]].get('reverse'))

    def project(self,p,previous,behind=.6,ahead=3.,stop_s=None):
        lo=max(0,bisect.bisect_left(self.stations,max(0,previous-behind))-1)
        hi=min(len(self.points)-1,bisect.bisect_right(self.stations,previous+ahead))
        if stop_s is not None:hi=min(hi,bisect.bisect_left(self.stations,stop_s))
        best=(float('inf'),previous)
        for i in range(lo,hi):
            a,b=self.points[i:i+2];dx,dy=b[0]-a[0],b[1]-a[1];d=dx*dx+dy*dy
            t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/max(1e-12,d)))
            error=math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
            if error<best[0]:best=(error,self.stations[i]+t*(self.stations[i+1]-self.stations[i]))
        return best

    def speed_cap(self,s):
        s=max(0,min(self.length,s))
        i=min(len(self.stations)-2,max(0,bisect.bisect_right(self.stations,s)-1))
        fraction=(s-self.stations[i])/max(1e-9,self.stations[i+1]-self.stations[i])
        return math.sqrt(max(0,(1-fraction)*self.speed_caps[i]**2+fraction*self.speed_caps[i+1]**2))

    def steering(self,p,heading,s,speed=0.):
        index=self.edge_index(s);e=self.model.edges[self.edges[index]]
        # A finite endpoint makes pure pursuit chase a nearby point without
        # controlling the final body orientation. Align to the service pose
        # over the final six metres, with signed lateral feedback in reverse.
        # Only use this after the last gear change and on service access.
        if (hasattr(self,'arrival_heading') and self.length-s<6.
                and e['kind']=='service_access'
                and all(bool(self.model.edges[k].get('reverse'))==self.reverse(s)
                        for k in self.edges[index:])):
            desired=self.arrival_heading;end=self.point(self.length)
            error=(heading-desired+math.pi)%(2*math.pi)-math.pi
            lateral=-(p[0]-end[0])*math.sin(desired)+(p[1]-end[1])*math.cos(desired)
            direction=-1 if self.reverse(s) else 1
            gain=1.8 if self.length-s<3. else 1.2
            delta=direction*math.atan(2.3285*(-gain*error-.5*lateral*direction))
            return max(-math.radians(40),min(math.radians(40),delta))
        lookahead=1.65 if e['kind']=='uturn' or e.get('maneuver')=='RIGHT' else 2.7
        if (self.model.raw.get('speed_adaptive_steering',False)
                and (e['kind']=='road' or e.get('maneuver')=='STRAIGHT') and not self.reverse(s)):
            lookahead=max(lookahead,min(6.4,abs(speed)*.8))
            # Keep the existing preview at curves. A longer straight-road
            # preview must not start cutting into a validated tight turn.
            if not hasattr(self,'_steering_curve_starts'):
                self._steering_curve_starts=[self.edge_starts[i] for i,k in enumerate(self.edges)
                    if self.model.edges[k]['kind'] in ('uturn','lane_change','service_access')
                    or self.model.edges[k].get('maneuver') in ('LEFT','RIGHT')]
            j=bisect.bisect_right(self._steering_curve_starts,s)
            if j<len(self._steering_curve_starts):
                lookahead=min(lookahead,max(2.7,self._steering_curve_starts[j]-s))
        delayed=e.get('straight_until_m') and s-self.edge_starts[index]<e['straight_until_m']-.02
        calibrated=e.get('right_turn_tracking_gain',1.)
        if not math.isfinite(calibrated) or not 1.<=calibrated<=1.35:
            raise ValueError('Invalid right turn tracking gain')
        if delayed and calibrated==1.:return 0.
        reversing=self.reverse(s)
        if reversing:lookahead=1.6;heading+=math.pi
        changes=(index+1<len(self.edges) and bool(self.model.edges[self.edges[index+1]].get('reverse'))!=reversing)
        # Slices are storage boundaries, not steering or gear boundaries.
        # Preview the continuous reverse curve instead of repeatedly aiming at
        # each short slice endpoint (which causes late, abrupt corrections).
        aim_station=min(s+lookahead,self.edge_ends[index]) if changes else s+lookahead
        if delayed:
            # Preserve the straight approach, but allow measured lateral/yaw
            # error correction instead of blindly suppressing all steering.
            aim_station=min(aim_station,self.edge_starts[index]+e['straight_until_m'])
        # A preview may cross several <=5 m slices. Stop it at the next gear
        # boundary so a forward truck never steers toward a future reverse leg.
        if not hasattr(self,'_steering_gear_stations'):
            self._steering_gear_stations=[cp['s'] for cp in self.checkpoints if cp['event']=='GEAR_CHANGE']
        gear=bisect.bisect_right(self._steering_gear_stations,s+1e-7)
        if gear<len(self._steering_gear_stations):aim_station=min(aim_station,self._steering_gear_stations[gear])
        aim=self.point(aim_station)
        alpha=(math.atan2(aim[1]-p[1],aim[0]-p[0])-heading+math.pi)%(2*math.pi)-math.pi
        delta=max(-math.radians(40),min(math.radians(40),calibrated*math.atan2(2*2.3285*math.sin(alpha),max(.25,math.dist(aim,p)))))
        return -delta if reversing else delta
