#!/usr/bin/env python3
"""Probe whether two wedged trucks can clear each other along their own plans.

Usage: .venv/bin/python scripts/probe_wedge_geometry.py <checkpoint.pkl> A B

Measures, for both trucks, the margin/hard gaps along their planned forward
motion and the distance to contact, which is how round39 established that
Car_4_0's turn arc physically overlaps Car_5_1's lane.
"""
import pickle,sys,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_control import envelope
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps,separating_gap


def main(argv):
    if len(argv)<4:raise SystemExit(__doc__)
    blob=pickle.loads(Path(argv[1]).read_bytes())
    states=blob['states'];executions=blob['executor'].controller.executions
    a,b=argv[2],argv[3]
    print('pair',a,b,'gap35',round(separating_gap(envelope(states[a]),envelope(states[b]),.35),4),
          'gap02',round(separating_gap(envelope(states[a]),envelope(states[b]),.2),4))
    for who,other in ((a,b),(b,a)):
        e=executions[who];print('==',who,'forward along its own plan vs',other,'stopped body')
        contact=None;d=0.
        while d<25.:
            pt=e.plan.point(e.progress+d);h=e.plan.heading(e.progress+d)
            box=envelope(VehicleState(who,*pt,h))
            gap=separating_gap(box,envelope(states[other]),.35)
            if contact is None and overlaps(box,envelope(states[other]),.0):contact=d
            if d in (0.,.25,.5,1.,1.5,2.,3.,5.,10.,20.) or (contact is not None and abs(d-contact)<.13):
                print(f'   +{d:5.2f} m  ({pt[0]:8.3f},{pt[1]:8.3f})  gap35={gap:+.3f}')
            d+=.25
        print('   first real body overlap at +', None if contact is None else round(contact,2),'m')
    return 0


if __name__=='__main__':raise SystemExit(main(sys.argv))
