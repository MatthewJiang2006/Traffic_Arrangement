"""Run shared TransportController through the formal AirSim adapter."""
import json
import math
import time
from logistics_park.common.messages import ControlCommand, VehicleState
from logistics_park.safety.collision import OBB, overlaps
from .transport_control import TransportController, zone_box
from .ground_contact import supported_ramp_contact
from .transport_routes import TransportMap, RouteGeometry
from .cargo_visuals import CargoVisualEvents


def configured_plans(model, scenario):
    plans={}
    for name,row in scenario['routes'].items():
        model.graph.validate_route(tuple(row['edges']))
        checkpoints=row.get('checkpoints') or [dict(leg=0,site=row['stops'][0],node=model.edges[row['edges'][-1]]['to_node'],after_edge_count=len(row['edges']),event='ARRIVAL')]
        plans[name]=RouteGeometry(model,row['edges'],checkpoints,row['origin'],row['stops'])
    return plans


def observed_boxes(audit):
    if len(audit.get('bodies',[]))!=3:
        raise RuntimeError('incomplete three-part truck body observation')
    return [OBB(*b['centre_ue_m'][:2],math.radians(b['yaw_deg']),2*b['half_size_m'][0],2*b['half_size_m'][1]) for b in audit['bodies']]


def run_transport(adapter,scenario,observer,output):
    model=TransportMap(scenario['map']);plans=configured_plans(model,scenario)
    shared_enabled=scenario.get('shared_resources_enabled',True)
    controller=TransportController(model,plans,service_s=scenario.get('service_s',2),shared_resources=shared_enabled)
    report=dict(status='FAIL',failure=[],execution='formal_airsim_transport_controller',ue_physics_validated=False,camera_changed=False,scope=scenario.get('scope','two-way shared road channel; virtual endpoint service, no warehouse routing'),samples=[],ground_contact_samples=[])
    hold={n:0. for n in plans};sequence=0
    visuals=CargoVisualEvents(scenario['cargo_visuals']['events_path']) if scenario.get('cargo_visuals') else None
    def rear(state):
        d=adapter.config.wheelbase_m/2
        return VehicleState(state.vehicle_id,state.x-d*math.cos(state.heading),state.y-d*math.sin(state.heading),state.heading,speed=state.speed,timestamp=state.timestamp)
    try:
        states=adapter.read_states()
        baseline={n:adapter.telemetry[n]['collision']['time_stamp'] for n in plans}
        for n in plans:
            if math.dist((rear(states[n]).x,rear(states[n]).y),plans[n].point(0))>.2 or abs(states[n].speed)>.1:
                raise RuntimeError('vehicles must start stationary at configured scenario positions')
            expected=plans[n].heading(0)
            if abs((states[n].heading-expected+math.pi)%(2*math.pi)-math.pi)>.05:
                raise RuntimeError('vehicle heading mismatch '+n)
            controller.activate(n,0.)
        begin=last=time.monotonic();last_print=begin
        while time.monotonic()-begin<scenario.get('max_runtime_s',180):
            now=time.monotonic();t=now-begin;dt=now-last;last=now
            measured=adapter.read_states();states={n:rear(s) for n,s in measured.items()}
            audits={};boxes={}
            for n in plans:
                audit,errors=observer.read(n);audits[n]=audit;boxes[n]=observed_boxes(audit)
                if errors:raise RuntimeError(n+': '+str(errors))
                hit=adapter.telemetry[n]['collision']
                if hit['has_collided'] and hit['time_stamp']!=baseline[n]:
                    row=dict(collision=hit['object_name'],
                        collision_normal=[hit['normal'][k] for k in ('x_val','y_val','z_val')],
                        collision_penetration_m=hit['penetration_depth'],
                        wheel_clearances_m=[w['clearance_m'] for w in audit['wheels']],
                        wheel_surfaces=[w['road_actor'] for w in audit['wheels']],
                        body_hits=[b['static_sweep_hit'] for b in audit['bodies']],
                        parked_vehicle_overlaps=[b['parked_vehicle_overlap'] for b in audit['bodies']])
                    ground=set(scenario.get('verified_ground_actors',[]))
                    ground.update(s for s in row['wheel_surfaces'] if s and s.startswith(('SM_Road_','SM_Floor_')))
                    if not supported_ramp_contact(row,ground,clearance_m=None):
                        raise RuntimeError(n+' native collision '+hit['object_name'])
                    baseline[n]=hit['time_stamp']
                    report['ground_contact_samples'].append(dict(t_s=t,vehicle_id=n,**row))
                if not .5<adapter.telemetry[n]['ue_z_m']<1.55:raise RuntimeError('chassis height '+n)
            # Include finished trucks in physical overlap checks; do not hide stopped bodies.
            names=list(plans)
            for i,n in enumerate(names):
                for m in names[i+1:]:
                    if any(overlaps(a,b,.05) for a in boxes[n] for b in boxes[m]):raise RuntimeError('physical body overlap '+n+' '+m)
            commands=controller.tick(states,t,body_boxes=boxes)
            if visuals:visuals.update(controller.events)
            for n in plans:
                target,delta=commands.get(n,(0.,0.))
                omega=0 if abs(target)<.05 else target*math.tan(delta)/adapter.config.wheelbase_m
                adapter.send(ControlCommand(n,sequence,adapter.clock(),target,omega,ttl=adapter.config.command_ttl_s))
                queued=bool(controller.shared_scheduler and any(n in controller.shared_scheduler.queued(ch) for ch in model.shared_resources))
                if queued and abs(states[n].speed)<.08:hold[n]+=dt
            sequence+=1
            areas={ch:zone_box(v) for ch,v in model.shared_resources.items()} if shared_enabled else {}
            owners=dict(controller.shared_scheduler.owners) if controller.shared_scheduler else {}
            queues={ch:list(controller.shared_scheduler.queued(ch)) for ch in model.shared_resources} if controller.shared_scheduler else {}
            report['samples'].append(dict(t_s=t,positions={n:[s.x,s.y] for n,s in states.items()},speeds_m_s={n:s.speed for n,s in states.items()},owners=owners,queued=queues,body_audits=audits,berth_owners={key:list(station.occupants) for key,station in controller.services.items()},channel_body_overlap={ch:{n:any(overlaps(b,area,.1) for b in boxes[n]) for n in plans} for ch,area in areas.items()}))
            if now-last_print>5:
                print('channel:',owners,'queued:',queues,'finished:',sum(ex.finished for ex in controller.executions.values()),flush=True);last_print=now
            if all(ex.finished for ex in controller.executions.values()):break
            time.sleep(.1)
        else:raise RuntimeError('shared UE transport timeout')
        entered=[e['vehicle_id'] for e in controller.events if e['event']=='SHARED_RESOURCE_ENTERED']
        released=[e['vehicle_id'] for e in controller.events if e['event']=='SHARED_RESOURCE_RELEASED']
        expected=scenario.get('expected_shared_order')
        if shared_enabled and ((expected is not None and entered!=expected) or sorted(released)!=sorted(entered)):
            raise RuntimeError('missing shared entry/release or FIFO violation')
        if shared_enabled and max(hold.values())<scenario.get('minimum_wait_s',2):raise RuntimeError('missing stationary queued-wait evidence')
        if controller.shared_scheduler and controller.shared_scheduler.owners:raise RuntimeError('shared owner remains')
        if scenario.get('require_parallel_motion') and controller.maximum_moving<2:
            raise RuntimeError('missing simultaneous two-vehicle motion')
        if controller.report()['status']!='PASS':raise RuntimeError('transport tasks incomplete')
        if model.raw.get('service_stations'):
            if any(ex.cargo_units for ex in controller.executions.values()):raise RuntimeError('warehouse cargo not discharged')
            if any(station.occupants for station in controller.services.values()):raise RuntimeError('warehouse berth not physically cleared')
            if scenario.get('interwarehouse'):
                report['interwarehouse_deliveries_completed']=sum(e['event']=='UNLOADING_COMPLETE' for e in controller.events)
            else:
                report['warehouse_local_missions_completed']=len(plans)
        report.update(status='PASS',ue_physics_validated=True)
    except Exception as exc:
        report['failure'].append(repr(exc));adapter.fail_stop('shared transport: '+str(exc));print('Shared UE FAIL:',exc,flush=True)
    finally:
        adapter.close()
        report.update(controller=controller.report(),stationary_shared_wait_s=hold,adapter_fault=adapter.fault,adapter_close_result=adapter.close_result,adapter_closed=adapter.closed)
        if adapter.fault or not adapter.close_result.get('api_restored'):
            report.update(status='FAIL',ue_physics_validated=False)
        output.mkdir(parents=True,exist_ok=True)
        (output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
