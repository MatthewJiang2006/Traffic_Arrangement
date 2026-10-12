"""Bind congestion prechecks to the algorithm and execution code as well as maps."""
import hashlib

def algorithm_fingerprints(root):
    files=(
        'src/logistics_park/scheduling/maneuver_admission.py',
        'src/logistics_park/runtime/safe_landing.py',
        'src/logistics_park/scheduling/efficiency_policy.py',
        'src/logistics_park/scheduling/travel_time.py',
        'src/logistics_park/runtime/wait_cycle_recovery.py',
        'src/logistics_park/scheduling/traffic_routing.py',
        'src/logistics_park/scheduling/local_traffic.py',
        'src/logistics_park/scheduling/conflict_timing.py',
        'src/logistics_park/runtime/speed_limits.py',
        'src/logistics_park/runtime/traffic_policy.py',
        'src/logistics_park/runtime/conflict_detour.py',
        'src/logistics_park/runtime/diagnostic_precheck.py',
        'src/logistics_park/scheduling/task_dispatch.py',
        'src/logistics_park/scheduling/shared_resource.py',
        'src/logistics_park/routing/service_network.py',
        'src/logistics_park/runtime/task_execution.py',
        'src/logistics_park/runtime/queue_clearance.py',
        'src/logistics_park/runtime/fleet_progress.py',
        'src/logistics_park/runtime/transport_routes.py',
        'src/logistics_park/runtime/transport_control.py',
        'scripts/simulate_ue_continuous_tasks.py',
        'scripts/validate_ue_continuous_tasks.py',
        'scripts/ue/monitor_short_cargo.py',
        'scripts/ue/monitor_fleet.py',
        'scripts/ue/node_pair_fleet_fixture.py',
        'scripts/ue/sample_fleet_wheels.py',
        'src/logistics_park/adapters/airsim.py',
        'src/logistics_park/runtime/body_observer.py',
        'src/logistics_park/scheduling/wait_dependencies.py',
        'src/logistics_park/safety/collision.py',
        'src/logistics_park/runtime/run_lock.py',
        'src/logistics_park/runtime/ground_contact.py',
    )
    return {name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in files}
