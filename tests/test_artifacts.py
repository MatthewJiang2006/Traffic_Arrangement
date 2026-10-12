import csv
import json
from dataclasses import replace

from logistics_park.simulation.lightweight import cli
from logistics_park.simulation.lightweight.simulator import Simulator
from logistics_park.simulation.lightweight.visualization import write_svg


def test_artifacts_include_config_and_world_trajectory(graph,config,tmp_path):
    sim = Simulator(graph,replace(config,duration=1.0),1)
    result = sim.run()
    assert result["status"].endswith("_FAIL")
    sim.write_artifacts(tmp_path,result)
    assert json.loads((tmp_path/"summary.json").read_text()) == result
    assert json.loads((tmp_path/"effective_config.json").read_text())["dt"] == 0.02
    with (tmp_path/"trajectory.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert float(rows[0]["timestamp"]) == 0
    assert float(rows[-1]["timestamp"]) == 1
    assert len(rows) == 21
    write_svg(graph,sim.rows,config.vehicle.length,config.vehicle.width,tmp_path/"plot.svg")
    import xml.etree.ElementTree as ET
    assert ET.parse(tmp_path/"plot.svg").getroot().tag.endswith("svg")


def test_cli_returns_nonzero_on_failed_acceptance(config,tmp_path,monkeypatch):
    from conftest import ROOT
    data = json.loads((ROOT/"config/simulation/minimal.json").read_text())
    data["duration"] = 0.2
    path = tmp_path/"config.json"
    path.write_text(json.dumps(data))
    monkeypatch.setattr("sys.argv",["logistics-park","--map",str(ROOT/"config/topology/minimal_loop.json"),
                                  "--config",str(path),"--output",str(tmp_path/"run")])
    assert cli.main() == 1
    assert (tmp_path/"run"/"map.json").exists()
