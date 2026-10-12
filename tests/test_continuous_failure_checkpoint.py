"""Failed light runs must retain the observed failure frame for diagnosis."""
import json,pickle
from pathlib import Path


def test_tick_failure_saves_latest_observation_checkpoint(tmp_path,monkeypatch):
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    import simulate_ue_continuous_tasks as sim
    from logistics_park.runtime.task_execution import ContinuousTaskExecutor
    scenario=root/'config/runtime/industrialarea_ue_fleet16_round47_proactive_full_v21.json'
    spec=json.loads(scenario.read_text());spec['output']=str(tmp_path)
    monkeypatch.setattr(sim,'SCENARIO_PATH',scenario)
    monkeypatch.setattr(sim,'SPEC',spec)
    monkeypatch.setattr(sim,'POSITIONS',spec['positions'])
    monkeypatch.setattr(sim,'TASKS',spec['tasks'])
    original=ContinuousTaskExecutor.tick;observed={}
    def failed_tick(self,states,now):
        if now<.2:return original(self,states,now)
        name=next(iter(states));states[name].x+=.12;states[name].timestamp=now
        observed[name]=states[name].x
        raise RuntimeError('synthetic tick observation fault')
    monkeypatch.setattr(ContinuousTaskExecutor,'tick',failed_tick)
    assert sim.run()==1
    report=json.loads((tmp_path/'lightweight_acceptance.json').read_text())
    snapshot=pickle.loads((tmp_path/'checkpoint.pkl').read_bytes())
    assert report['status']=='FAIL' and 'synthetic tick observation fault' in report['failure']
    assert snapshot['now']==.2
    for name,x in observed.items():
        assert snapshot['states'][name].x==x and snapshot['states'][name].timestamp==.2
    assert len(snapshot['states'])==16
