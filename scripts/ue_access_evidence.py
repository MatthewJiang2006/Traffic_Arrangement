"""Bind prior access evidence to saved maps and verified scene changes."""
import json
from pathlib import Path


def unchanged_access_region(record,stem,current_maps):
    root=Path(__file__).resolve().parents[1]/"runs/ue_scene/station_siting"
    for filename,flag in (("access_regions_after_ramp_seam_repair.json","unchanged"),
                          ("previous_access_regions_after_waiting_patch.json","unchanged"),
                          ("waiting_fence_removal_preservation.json","clearance_preserved")):
        path=root/filename
        if not path.exists():
            continue
        audit=json.loads(path.read_text())
        if (audit.get("cases",{}).get(stem,{}).get("before_map_sha256",audit.get("before_map_sha256"))==record.get("map_sha256")
                and audit.get("after_map_sha256")==current_maps
                and audit.get("cases",{}).get(stem,{}).get(flag,False)
                and (flag=="unchanged" or audit.get("obstacles_only_removed_verified",False))):
            return True
    return False
