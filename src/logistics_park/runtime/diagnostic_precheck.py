"""Bounded diagnostic runs are distinct from full transport acceptance."""
import math


def prefix_precheck_ready(report, vehicle_count, requested_seconds):
    duration=report.get('diagnostic_preflight_s',0)
    elapsed=report.get('elapsed_sim_s',0)
    return bool(report.get('status')=='DIAGNOSTIC_READY'
        and report.get('failure') is None
        and isinstance(duration,(int,float)) and math.isfinite(duration) and duration>=60
        and duration==requested_seconds
        and isinstance(elapsed,(int,float)) and math.isfinite(elapsed) and elapsed>=duration
        and report.get('vehicle_count')==vehicle_count
        and all(report.get(key)==0 for key in ('collision_count','unauthorized_shared_entry_count',
            'local_unauthorized_entry_count','road_capacity_violation_steps')))
