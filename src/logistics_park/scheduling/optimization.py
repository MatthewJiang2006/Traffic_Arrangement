"""Optional advisory scheduling; movement still requires existing admission."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SchedulingOptions:
    parking_entry_priority: bool = False
    transport_entry_priority: bool = False
    tail_prepositioning: bool = False
    continuation_dispatch: bool = False
    pickup_tail_weight: float = 0.
    submission_order_dispatch: bool = False
    calibrated_eta: bool = False
    batch_dispatch: bool = False
    waiting_completion_cost: bool = False
    parking_lookahead: bool = False
    parking_tail_reserve: bool = False
    connected_clearance: bool = False
    protect_granted_queues: bool = False
    max_loading_inflight: int = 0
    decision_interval_s: float = 2.
    berth_intent_hysteresis_s: float = 0.
    traffic_forecast: bool = False
    rolling_scheduler: bool = False
    calendar_departure_wait: bool = False
    planning_interval_s: float = 5.
    planning_budget_s: float = .02

    def __post_init__(self):
        for key in ('tail_prepositioning','transport_entry_priority','parking_entry_priority','continuation_dispatch','submission_order_dispatch','calibrated_eta','batch_dispatch','waiting_completion_cost','parking_lookahead','parking_tail_reserve','connected_clearance','protect_granted_queues'):
            if not isinstance(getattr(self,key),bool):raise ValueError('Invalid scheduling flag '+key)
        for key in ('traffic_forecast','rolling_scheduler','calendar_departure_wait'):
            if not isinstance(getattr(self,key),bool):raise ValueError('Invalid scheduling flag '+key)
        for key in ('planning_interval_s','planning_budget_s'):
            value=getattr(self,key)
            if isinstance(value,bool) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid planning timing '+key)
        if isinstance(self.pickup_tail_weight,bool) or not math.isfinite(self.pickup_tail_weight) or not 0<=self.pickup_tail_weight<=1:
            raise ValueError('Invalid pickup tail weight')
        if type(self.max_loading_inflight) is not int or self.max_loading_inflight<0:
            raise ValueError('Invalid loading admission limit')
        if not math.isfinite(self.decision_interval_s) or self.decision_interval_s<=0:
            raise ValueError('Invalid scheduling interval')
        if (isinstance(self.berth_intent_hysteresis_s,bool)
                or not math.isfinite(self.berth_intent_hysteresis_s)
                or not 0<=self.berth_intent_hysteresis_s<=30):
            raise ValueError('Invalid berth intent hysteresis')


def bottleneck_assignment(costs):
    """Maximum cardinality, then smallest latest finish, then total cost.

    Infinite pairs are unavailable. Rows and columns are used at most once.
    No reservations or vehicle state are changed by this calculation.
    """
    n=len(costs)
    if not n:return []
    m=len(costs[0])
    if any(len(row)!=m for row in costs):raise ValueError('Ragged assignment matrix')
    if any(math.isnan(v) or v<0 for row in costs for v in row):raise ValueError('Invalid assignment cost')
    values=sorted({v for row in costs for v in row if math.isfinite(v)})
    if not values:return []
    def cardinality(limit):
        owners={}
        def augment(i,seen):
            for j,v in enumerate(costs[i]):
                if v>limit or j in seen:continue
                seen.add(j)
                if j not in owners or augment(owners[j],seen):
                    owners[j]=i;return True
            return False
        return sum(augment(i,set()) for i in range(n))
    wanted=cardinality(values[-1]);lo=0;hi=len(values)-1
    while lo<hi:
        mid=(lo+hi)//2
        if cardinality(values[mid])>=wanted:hi=mid
        else:lo=mid+1
    limit=values[lo]
    # Private dummy columns allow unmatched rows. One dummy costs more than
    # the total of all real assignments, preserving maximum cardinality.
    dummy=(limit+1)*(n+1);forbidden=dummy*(n+1)
    a=[[v if v<=limit else forbidden for v in row]+[dummy]*n for row in costs]
    width=m+n;u=[0.]*(n+1);v=[0.]*(width+1);p=[0]*(width+1);way=[0]*(width+1)
    for i in range(1,n+1):
        p[0]=i;j0=0;minimum=[math.inf]*(width+1);used=[False]*(width+1)
        while True:
            used[j0]=True;i0=p[j0];delta=math.inf;j1=0
            for j in range(1,width+1):
                if used[j]:continue
                cur=a[i0-1][j-1]-u[i0]-v[j]
                if cur<minimum[j]:minimum[j]=cur;way[j]=j0
                if minimum[j]<delta:delta=minimum[j];j1=j
            for j in range(width+1):
                if used[j]:u[p[j]]+=delta;v[j]-=delta
                else:minimum[j]-=delta
            j0=j1
            if p[j0]==0:break
        while True:
            j1=way[j0];p[j0]=p[j1];j0=j1
            if j0==0:break
    return sorted((p[j]-1,j-1) for j in range(1,m+1)
                  if p[j] and costs[p[j]-1][j-1]<=limit)
