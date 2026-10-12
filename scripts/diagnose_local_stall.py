#!/usr/bin/env python3
"""Inspect the exact local state at a light-simulation stall checkpoint.

Usage: .venv/bin/python scripts/diagnose_local_stall.py <checkpoint.pkl> [vehicle ...]

Prints each focus vehicle's progress, holds, leases, denials and the pairwise
OBB gaps, which is what identified round39's Car_4_0 / Car_5_1 wedge.
"""
import pickle,sys,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_control import envelope
from logistics_park.scheduling.conflict_timing import maneuver_groups
from logistics_park.safety.collision import overlaps,separating_gap


def main(argv):
    if len(argv)<2:raise SystemExit(__doc__)
    blob=pickle.loads(Path(argv[1]).read_bytes())
    ex=blob['executor'];controller=ex.controller;local=controller.local_traffic
    states=blob['states'];now=blob['now'];executions=controller.executions
    focus=argv[2:] or sorted(states)
    print('checkpoint now',now)
    if not hasattr(local,'action_groups'):
        local.action_groups={n:maneuver_groups(e.plan) for n,e in executions.items()}
    for n in focus:
        s=states[n];e=executions[n]
        print('==',n,'pos',round(s.x,3),round(s.y,3),'heading',round(s.heading,3),'v',round(s.speed,3))
        print('   progress',round(e.progress,3),'finished',e.finished,'reverse',e.plan.reverse(e.progress),
              'edge',e.plan.edges[e.plan.edge_index(e.progress)])
        print('   lane groups',[(g[0],round(g[2],2),round(g[3],2)) for g in local.action_groups.get(n,())])
        print('   destination hold',controller.destination_holds.get(n),
              'maneuver hold',local.maneuver_holds.get(n),'yield hold',local.yield_holds.get(n))
        print('   paused_by',local.paused_by.get(n),'clearance_pauses',local.clearance_pauses.get(n))
        print('   leases',[(k[1],round(v['end'],3)) for k,v in local.passages.items() if k[0]==n])
        print('   denials',local.last_denials.get(n),'passage_denials',local.passage_denials.get(n))
    print('== pairwise OBB gaps at margin 0.35 / hard 0.2')
    for i,a in enumerate(focus):
        for b in focus[i+1:]:
            print('  ',a,b,'gap35',round(separating_gap(envelope(states[a]),envelope(states[b]),.35),4),
                  'gap02',round(separating_gap(envelope(states[a]),envelope(states[b]),.2),4),
                  'overlap20',overlaps(envelope(states[a]),envelope(states[b]),.2))
    return 0


if __name__=='__main__':raise SystemExit(main(sys.argv))
