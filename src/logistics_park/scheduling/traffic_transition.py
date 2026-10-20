"""Pure feasibility checks shared by every right-of-way state transition.

A transition describes obligations, not just edits to a permission dictionary.
The caller submits it before changing plans, leases or precedence. Geometry
checks here use the same rear-axle stations and complete bodies as execution.
This is bounded local feasibility, not a proof of global fleet liveness.
"""
from dataclasses import dataclass
import math
from copy import copy
from logistics_park.common.messages import VehicleState

from logistics_park.runtime.safety_timing import braking_distance
from .conflict_timing import maneuver_groups
from .wait_dependencies import strongly_connected_cycles

from .maneuver_contract import (TAIL_CLEAR_M, SWEEP_STEP_M, ActionContract,
                                holding_line, sample_sweep)


@dataclass(frozen=True)
class TrafficTransition:
    kind: str
    actor: str
    handoffs: tuple = ()         # (finished vehicle, next-leg plan)
    routes: tuple = ()           # (vehicle, replacement plan)
    revoke: tuple = ()           # (vehicle, action start)
    retire: tuple = ()           # completed, already cleared leases only
    grants: tuple = ()           # (key, lease)
    waits: tuple = ()            # (yielding vehicle, passing vehicle)
    holds: tuple = ()            # (vehicle, required stop station)
    clearances: tuple = ()       # (paused owner, passing vehicle, required end)
    resolved: tuple = ()         # previously denied requests rechecked by caller
    compatible: tuple = ()      # temporal compatibility with existing grants


@dataclass(frozen=True)
class TransitionDecision:
    allowed: bool
    reason: str = ''
    blocked_by: str = ''


def wait_rows(local, states, proposal):
    rows = []
    for row in getattr(local, 'admission_dependencies', ()):
        a, b = row['vehicle'], row['blocked_by']
        if any(a == n and row.get('resource') == 'passage:' + str(start)
               for n, start in proposal.resolved):
            continue
        if a in states and b in states and abs(states[a].speed) < .15:
            rows.append(dict(vehicle=a, blocked_by=b))
    for owner, keys in local.paused_by.items():
        rows.extend(dict(vehicle=owner, blocked_by=key[0]) for key in keys
                    if key not in proposal.revoke and key not in proposal.retire)
    for owner, grantees in local.clearance_pauses.items():
        rows.extend(dict(vehicle=owner, blocked_by=n) for n in grantees)
    return rows



def wait_cycle(local, states, proposal, additional=()):
    added={(a,b) for a,b in (*proposal.waits,*additional) if a!=b}
    rows=wait_rows(local,states,proposal)
    rows.extend(dict(vehicle=a,blocked_by=b) for a,b in added)
    return next((cycle for cycle in strongly_connected_cycles(rows)
                 if any(a in cycle and b in cycle for a,b in added)),None)


class TransitionValidator:
    """Read one coherent controller frame; never mutate its collaborators."""

    def __init__(self, local, executions, states, boxes, envelope):
        self.local = local
        self.executions = executions
        self.states = states
        self.boxes = boxes
        self.envelope = envelope
        self.margin = local.margin_m
        self.step = local.step_s

    def deny(self, reason, other=''):
        return TransitionDecision(False, reason, other)

    def progress(self,name):
        ex=self.executions[name];plan=ex.plan;state=self.states[name]
        if not hasattr(plan,'project'):return ex.progress
        cp=plan.checkpoints[ex.checkpoint] if ex.checkpoint<len(plan.checkpoints) else None
        options=dict(ahead=max(3.,abs(state.speed)*.1+2.))
        if cp and cp['event'] in ('ARRIVAL','GEAR_CHANGE'):options['stop_s']=cp['s']
        gears=[c['s'] for c in plan.checkpoints[:ex.checkpoint] if c['event']=='GEAR_CHANGE']
        if gears:options['start_s']=gears[-1]
        _,station=plan.project((state.x,state.y),ex.progress,**options)
        if cp and cp['event']=='GEAR_CHANGE':station=min(station,max(0.,cp['s']-1e-6))
        return max(ex.progress,station)

    def can_stop(self, name, station, *, stationary_hold=False):
        state = self.states[name]
        # The 7.8m holding boundary already contains nose/clearance room.
        # braking_distance's default .5m is following/approach headroom;
        # adding it again turns a legally settled pose into an irrevocable
        # promise. Keep reaction/deceleration and sampled acceleration here.
        required=braking_distance(state.speed,margin_m=0.)
        if not (stationary_hold and abs(state.speed)<.08):
            required+=.3*self.step**2
        return station-self.progress(name)+1e-6>=required

    def lease_phase(self, key, lease):
        name, start = key
        ex = self.executions[name]
        if ex.finished or (ex.plan is lease['plan'] and ex.progress > lease['end'] + TAIL_CLEAR_M):
            return 'cleared'
        if ex.plan is not lease['plan']:
            return 'stale'
        if self.progress(name) >= ex.plan.edge_starts[start]:
            return 'entered'
        if not self.can_stop(name, holding_line(ex.plan.edge_starts[start])):
            return 'committed'
        return 'granted'

    def _intersects(self, one, two):
        # Import lazily to avoid the LocalTraffic/validator import cycle.
        from .local_traffic import sweeps_intersect
        return sweeps_intersect(one, two, self.margin)

    def _route_obligations(self, name, plan):
        ex = self.executions[name]
        old = ex.plan
        progress=self.progress(name)
        if any(key[0] == name for key in self.local.passages):
            return self.deny('route_has_live_passage', name), []
        limit = progress + max(TAIL_CLEAR_M, braking_distance(self.states[name].speed))
        if plan.length <= limit:
            return self.deny('route_omits_braking_prefix', name), []
        tolerance = .001 if getattr(plan.model, 'raw', {}).get('point_conflicts', False) else 1e-6
        for i in range(math.ceil(limit / SWEEP_STEP_M) + 1):
            station = min(limit, i * SWEEP_STEP_M)
            if (math.dist(plan.point(station), old.point(station)) > tolerance
                    or plan.reverse(station) != old.reverse(station)):
                return self.deny('route_changes_braking_prefix', name), []
        if math.dist((self.states[name].x, self.states[name].y), plan.point(progress)) > .5:
            return self.deny('route_changes_measured_pose', name), []
        if not hasattr(plan, 'edge_starts'):
            return TransitionDecision(True), []
        old_signatures = {ActionContract.for_group(old, g).signature(old)
                          for g in maneuver_groups(old)} if hasattr(old, 'edge_starts') else set()
        waits = []
        for group in maneuver_groups(plan):
            action = ActionContract.for_group(plan, group, self.local.compact_permissions)
            if action.entry + .2 < progress < action.exit + .2:
                return self.deny('retroactively_required_maneuver', name), []
            if action.exit <= progress:
                continue
            if action.signature(plan) not in old_signatures and not self.can_stop(name, action.stop, stationary_hold=True):
                return self.deny('new_action_stop_line_committed', name), []
            # Distant uncommitted actions will be revalidated on admission.
            if action.entry > progress + max(25., braking_distance(self.states[name].speed)):
                continue
            if action.signature(plan) in old_signatures:
                continue
            sweep = [b for _, b in sample_sweep(plan, progress, action.outlet, self.envelope, name)]
            for other, state in self.states.items():
                if other == name:continue
                other_ex=copy(self.executions[other]);other_ex.progress=self.progress(other)
                occupied=list(self.boxes.get(other,[self.envelope(state)]))
                forecast=self.local._forecast(other,state,other_ex,0.,
                    getattr(self.local,'holds',{}).get(other),self.envelope)[0]
                occupied.extend(forecast)
                if not self._intersects(sweep, occupied):
                    continue
                # A stopped blocker needs an actual forward clearance witness
                # while the applicant remains at its measured pose. Merely
                # adding a wait edge does not establish an executable escape.
                if (other_ex.finished or self.local.paused_by.get(other)
                        or self.local.clearance_pauses.get(other)):
                    return self.deny('route_clearance_not_executable', other), []
                end_limit=min(other_ex.plan.length,other_ex.progress+40.,
                              self.local._clearance_stop(other,other_ex)[0])
                if end_limit<=other_ex.progress:
                    return self.deny('route_clearance_not_executable', other), []
                conflicts=[station for station,body in sample_sweep(
                    other_ex.plan,other_ex.progress,end_limit,self.envelope,other)
                    if self._intersects([body],sweep)]
                escape=(max(conflicts) if conflicts else other_ex.progress)+.5
                if escape>end_limit or self._intersects(
                        [self.envelope(VehicleState(other,*other_ex.plan.point(escape),other_ex.plan.heading(escape)))],sweep):
                    escape=None
                if (escape is None or not self.local._clearance_executable(
                        name,other,other_ex,self.states,self.envelope,escape)):
                    return self.deny('route_clearance_not_executable', other), []
                waits.append((name, other))
        return TransitionDecision(True), waits

    def validate(self, proposal):
        for name,plan in proposal.handoffs:
            ex=self.executions[name];state=self.states[name]
            if (not ex.finished or abs(state.speed)>.1
                    or math.dist((state.x,state.y),plan.point(0))>.5
                    or abs((state.heading-plan.heading(0)+math.pi)%(2*math.pi)-math.pi)>.35):
                return self.deny('next_leg_pose_not_preserved',name)
            old_rights={key for key in self.local.passages if key[0]==name}
            if not old_rights.issubset(set(proposal.retire)):
                return self.deny('next_leg_retains_stale_permission',name)
        for key in proposal.retire:
            lease = self.local.passages.get(key)
            if lease is None or self.lease_phase(key, lease) != 'cleared':
                return self.deny('retiring_uncleared_passage', key[0])
        for key in proposal.revoke:
            lease = self.local.passages.get(key)
            if lease is None or self.lease_phase(key, lease) != 'granted':
                return self.deny('passage_stop_line_committed', key[0])
        for name, stop in proposal.holds:
            if not self.can_stop(name, stop, stationary_hold=True):
                return self.deny('yield_stop_not_executable', name)
        new_waits = list(proposal.waits)
        for name, plan in proposal.routes:
            result, waits = self._route_obligations(name, plan)
            if not result.allowed:
                return result
            new_waits.extend(waits)
        for key, lease in proposal.grants:
            name = key[0]
            if name not in self.executions or lease['plan'] is not self.executions[name].plan:
                return self.deny('grant_route_version_mismatch', name)
            for other, physical in self.boxes.items():
                if other == name:
                    continue
                if self._intersects(lease['sweep'], physical):
                    return self.deny('passage_occupied_body', other)
                forecast = getattr(self.local, 'braking', {}).get(other)
                if other not in proposal.compatible and forecast and self._intersects(lease['sweep'], forecast[0]):
                    return self.deny('passage_braking_path', other)
        # A clearance is a concrete duty with a reachable endpoint, not a claim
        # that a dependency cycle will somehow disappear at a later time.
        for owner, name, end in proposal.clearances:
            ex = self.executions[name]
            if (self.local._clearance_stop(name, ex)[0] < end
                    or not self.local._clearance_executable(owner, name, ex, self.states, self.envelope, end)):
                return self.deny('clearance_not_executable', name)
        cycle=wait_cycle(self.local,self.states,proposal,new_waits)
        if cycle:
            return self.deny('prospective_wait_cycle', next(n for n in cycle if n!=proposal.actor))
        return TransitionDecision(True)
