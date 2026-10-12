#!/usr/bin/env python3
"""Restore the fleet after an interrupted node-pair sweep.

Mirrors the validator's own teardown: brake everything, reset the AirSim world,
run the fixture RESTORE that re-shows and re-enables the frozen trucks, zero all
motion, then hand the API back in its original disabled state.
"""
import json
import math
import sys
from pathlib import Path

import airsim

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ue_trailer_driving_check import ue_script  # noqa: E402

OUT = ROOT / "runs/ue_node_pairs"
NAMES = [f"Car_{i//3}_{i%3}" for i in range(12)] + [f"Car_{i//3}_{i%3}" for i in range(12, 24)]


def main():
    client = airsim.CarClient(timeout_value=10)
    names = client.listVehicles()
    print("vehicles", len(names))
    for name in names:
        try:
            client.enableApiControl(True, vehicle_name=name)
            client.setCarControls(airsim.CarControls(throttle=0, brake=1, handbrake=True), vehicle_name=name)
        except Exception as exc:
            print("brake failed", name, exc)
    client.reset()
    client.simPause(True)
    (OUT / "fleet_request.json").write_text(json.dumps(dict(action="RESTORE", vehicles=names, time_scale=1)))
    status = ue_script(client, "node_pair_fleet_fixture.py", OUT / "fleet_status.json")
    print("fixture", status)
    stop_dir = ROOT / "runs/ue_scene/test_sites"
    (stop_dir / "motion_stop_request.json").write_text(json.dumps(dict(vehicles=names)))
    print("stop", ue_script(client, "stop_test_vehicle_motion.py", stop_dir / "motion_stop.json"))
    client.simPause(False)
    for name in names:
        client.enableApiControl(False, vehicle_name=name)
    import time
    time.sleep(1.0)
    speeds = {n: abs(client.getCarState(vehicle_name=n).speed) for n in names}
    enabled = {n: client.isApiControlEnabled(vehicle_name=n) for n in names}
    report = dict(status="PASS", vehicles=len(names), max_speed_m_s=max(speeds.values()),
                  api_enabled_count=sum(enabled.values()), paused=client.simIsPause())
    (OUT / "recovery.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    raise SystemExit(main())
