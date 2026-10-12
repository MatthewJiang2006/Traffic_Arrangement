"""Arrival-only test lifecycle; normal transport still requires tail clearance."""
from .transport_control import envelope
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps


def can_stage(controller, plan):
    # Admit complete routes atomically. A truck waiting at a downstream
    # stop line must never block the exit of a region owned by another job.
    needed={v.channel_id for v in plan.shared_visits}
    for ex in controller.executions.values():
        remaining={v.channel_id for i,v in enumerate(ex.plan.shared_visits)
                   if i not in ex.shared_cleared}
        if needed & remaining:return False
    box=envelope(VehicleState('fixture',*plan.point(0),plan.heading(0)))
    if any(overlaps(box,b,.5) for boxes in controller.boxes.values() for b in boxes):return False
    for key,area in controller.shared_zones.items():
        if overlaps(box,area,.1):
            if key in controller.shared_scheduler.owners:return False
            if any(overlaps(b,area,.1) for boxes in controller.boxes.values() for b in boxes):return False
    return True


def clear_collected(controller,name,now,*,physically_removed):
    """Release test reservations only AFTER the arrived truck is removed."""
    if not controller.collect_at_destination or not physically_removed or not controller.executions[name].finished:
        raise ValueError('A stopped completed test truck must first be physically cleared')
    _clear_removed(controller,name,now)


def clear_aborted(controller,name,now,*,physically_removed):
    """Clear a failed test without inventing arrival or task completion."""
    if not controller.collect_at_destination or not physically_removed:
        raise ValueError('The failed test truck must first be physically cleared')
    controller.events.append(dict(event='TEST_PAIR_ABORTED',vehicle_id=name,time=now))
    _clear_removed(controller,name,now)


def _clear_removed(controller,name,now):
    for key,grant in list(controller.scheduler.owners.items()):
        if grant.request.vehicle_id==name:
            del controller.scheduler.owners[key]
            controller.events.append(dict(event='TEST_FIXTURE_RESOURCE_CLEARED',vehicle_id=name,time=now,resource=key))
    for queue in controller.scheduler.queues.values():queue.pop(name,None)
    if controller.shared_scheduler:
        for queue in controller.shared_scheduler.queues.values():queue.pop(name,None)
        for key,owner in list(controller.shared_scheduler.owners.items()):
            if owner==name:controller.shared_scheduler.release(key,name,now,tail_clear=True)
    for values in controller.capacity.occupants.values():values.discard(name)
    for values in controller.capacity.committed.values():values.discard(name)
    controller.boxes.pop(name,None);controller.states.pop(name,None)
    del controller.executions[name]
