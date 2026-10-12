"""Optional UE envelope/tire instrumentation for ground-truth acceptance."""
import json
from pathlib import Path
import time


class FileBodyObserver:
    def __init__(self,directory:Path,prefix='four',max_age_s=1.2,wheel_diagnostic_only=False):
        self.directory=directory;self.prefix=prefix;self.max_age_s=max_age_s
        self.wheel_diagnostic_only=wheel_diagnostic_only
    def read(self,name):
        row=json.loads((self.directory/(self.prefix+'_'+name+'_live.json')).read_text())
        if row.get('error'):raise RuntimeError('body observer '+row['error'])
        if not 0<=time.monotonic()-row['monotonic_s']<=self.max_age_s:raise RuntimeError('stale body observation '+name)
        errors=[]
        for b in row['bodies']:
            if b['static_sweep_hit']:errors.append(b['name']+' swept into '+b['static_sweep_hit'])
            if b['parked_vehicle_overlap']:errors.append(b['name']+' overlaps '+b['parked_vehicle_overlap'])
        for w in ([] if self.wheel_diagnostic_only else row['wheels']):
            if not (w['road_actor'] or '').startswith(('SM_Road_','Traffic_SeamBridge')):errors.append(w['name']+' off road')
            if w['clearance_m'] is None or abs(w['clearance_m'])>.08:errors.append(w['name']+' ground clearance')
        if abs(row['pitch_deg'])>8 or abs(row['roll_deg'])>8:errors.append('chassis attitude')
        return row,errors


class FleetFileObserver:
    """State and body audits from the same atomic UE frame."""
    def __init__(self,path,max_age_s=1.2,clock=time.monotonic):
        self.path=Path(path);self.max_age_s=max_age_s;self.clock=clock;self.latest=None
    def snapshot(self):
        row=json.loads(self.path.read_text())
        if row.get('error'):raise RuntimeError('fleet observer '+row['error'])
        if not 0<=self.clock()-row['monotonic_s']<=self.max_age_s:raise RuntimeError('stale fleet observation')
        if set(row['states'])!=set(row['audits']):raise RuntimeError('Incomplete fleet frame')
        self.latest=row;return row
    def read(self,name):
        if self.latest is None:self.snapshot()
        row=self.latest['audits'][name]
        errors=[]
        for body in row['bodies']:
            if body['static_sweep_hit']:errors.append(body['name']+' swept into '+body['static_sweep_hit'])
            if body['parked_vehicle_overlap']:errors.append(body['name']+' overlaps '+body['parked_vehicle_overlap'])
        if abs(row['pitch_deg'])>8 or abs(row['roll_deg'])>8:errors.append('chassis attitude')
        return row,errors
