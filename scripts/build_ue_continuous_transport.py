#!/usr/bin/env python3
"""Reuse road-scan slices; exclude roadside parking from road critical areas."""
import copy,json,sys,hashlib,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_control import envelope,zone_box
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps

def build():
    source=ROOT/'config/topology/industrialarea_ue_node_pair_tests.json'
    raw=json.loads(source.read_text())
    for name,spec in raw['shared_resources'].items():
        previous=list(spec['bounds'])
        if previous[0]<-70 and previous[2]<-70:spec['bounds'][2]=min(previous[2],-84.9)
        spec['original_test_bounds']=previous
        spec['description']='Physical critical area; parked trucks remain observed; only actual footprint clearance releases ownership'
        spec['edge_ids']=[]
        area=zone_box(spec)
        for e in raw['edges']:
            path=e['centerline']
            for i,p in enumerate(path):
                a,b=(p,path[i+1]) if i+1<len(path) else (path[i-1],p)
                h=math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if e.get('reverse') else 0)
                if overlaps(envelope(VehicleState('geometry',*p,h)),area,.8):
                    spec['edge_ids'].append(e['edge_id']);break
    raw['scope']='Continuous service execution; roadside waiting footprints excluded from west-road critical bounds'
    raw['source_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    target=ROOT/'config/topology/industrialarea_ue_continuous_transport.json'
    target.write_text(json.dumps(raw,indent=2)+'\n')
    print('CONTINUOUS MAP',len(raw['edges']),'edges',len(raw['shared_resources']),'regions')

if __name__=='__main__':build()
