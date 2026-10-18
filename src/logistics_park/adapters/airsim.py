"""AirSim car RPC backend, UE world coordinates, deadlines and fail-stop ownership."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable, Protocol
from logistics_park.common.messages import ControlCommand, VehicleState, finite
from logistics_park.runtime.safety_timing import NATIVE_CONTROL_TIMEOUT_S,MIN_ACTUATED_SPEED_M_S


class Transport(Protocol):
    def call(self, method: str, *args): ...
    def close(self): ...


def _decode_rpc_result(value, field=None):
    """Decode after unpacking so a malformed native label cannot poison a stream.

    Only collision object names tolerate invalid UTF-8; escaped bytes retain
    the evidence and cannot match an allowed road surface. Keys and other
    text remain strict.
    """
    if isinstance(value, bytes):
        return value.decode('utf-8', errors='backslashreplace' if field=='object_name' else 'strict')
    if isinstance(value, dict):
        result={_decode_rpc_result(k): _decode_rpc_result(v, _decode_rpc_result(k)) for k,v in value.items()}
        label=value.get(b'object_name',value.get('object_name'))
        if isinstance(label,bytes):
            try:label.decode('utf-8')
            except UnicodeDecodeError:result['object_name_utf8_valid']=False
        return result
    if isinstance(value, list):return [_decode_rpc_result(v) for v in value]
    return value


class RPCTransport:
    def __init__(self, host: str, port: int, timeout: float):
        import msgpackrpc  # Optional backend dependency; lightweight simulation needs none.
        self.client = msgpackrpc.Client(msgpackrpc.Address(host, port), timeout=timeout,
                                        pack_encoding='utf-8', unpack_encoding=None)
    def call(self, method, *args):
        return _decode_rpc_result(self.client.call(method, *args))
    def close(self):
        self.client.close()


@dataclass(frozen=True)
class AirSimConfig:
    vehicle_ids: tuple[str, ...]
    origin_ue_m: tuple[float, float, float]
    host: str = '127.0.0.1'
    port: int = 41451
    rpc_timeout_s: float = 2.0
    state_timeout_s: float = 1.2
    command_ttl_s: float = .8
    wheelbase_m: float = 2.3285
    max_steer_rad: float = math.radians(40)
    throttle_rise_per_s: float = .12
    speed_rise_m_s2: float = .5
    max_throttle: float = .45
    speed_error_gain: float = .3
    allow_reverse: bool = False

    def __post_init__(self):
        finite(*self.origin_ue_m, self.rpc_timeout_s, self.state_timeout_s, self.command_ttl_s,
               self.wheelbase_m, self.max_steer_rad, self.throttle_rise_per_s, self.speed_rise_m_s2,
               self.max_throttle, self.speed_error_gain)
        if not self.vehicle_ids or len(set(self.vehicle_ids)) != len(self.vehicle_ids) or any(not n for n in self.vehicle_ids):
            raise ValueError('unique nonempty vehicle IDs required')
        if len(self.origin_ue_m) != 3 or not 0 < self.port < 65536 or min(self.rpc_timeout_s, self.state_timeout_s,self.command_ttl_s,self.wheelbase_m,self.max_steer_rad,self.throttle_rise_per_s,self.speed_rise_m_s2,self.max_throttle,self.speed_error_gain) <= 0 or self.max_throttle > 1:
            raise ValueError('invalid AirSim configuration')


class AdapterFault(RuntimeError):
    pass


def controls(throttle=0.0, brake=0.0, steering=0.0):
    return dict(throttle=throttle, brake=brake, steering=steering, handbrake=False,
                is_manual_gear=False, manual_gear=0, gear_immediate=True)


class AirSimAdapter:
    def __init__(self, config: AirSimConfig, log_path: Path, *,
                 transport_factory: Callable[[], Transport] | None = None,
                 clock: Callable[[], float] = time.monotonic, watchdog: bool = True, snapshot_provider=None):
        self.config=config; self.clock=clock;self.snapshot_provider=snapshot_provider
        self.factory=transport_factory or (lambda: RPCTransport(config.host,config.port,config.rpc_timeout_s))
        self.transport=self.factory();self.lock=threading.RLock();self.log_lock=threading.Lock()
        log_path.parent.mkdir(parents=True,exist_ok=True);self.log=log_path.open('w',encoding='utf-8')
        self.telemetry={};self.states={};self.original_api={};self.last_source={};self.source_advanced={}
        self.state_received={};self.deadlines={};self.sequences={n:-1 for n in config.vehicle_ids}
        self.applied_speed={n:0.0 for n in config.vehicle_ids};self.throttle={n:0.0 for n in config.vehicle_ids}
        self.drive_direction={n:1 for n in config.vehicle_ids}
        self.last_send={n:clock() for n in config.vehicle_ids};self.fault=None;self.opened=False;self.closed=False
        self.close_result={};self.shutdown=threading.Event();self.thread=None;self.watchdog_enabled=watchdog

    def record(self,event,**fields):
        with self.log_lock:
            self.log.write(json.dumps(dict(event=event,monotonic_s=self.clock(),**fields),allow_nan=False)+'\n');self.log.flush()

    def open(self):
        if self.opened or self.closed:raise AdapterFault("adapter already opened or closed")
        try:
            if not self.transport.call('ping'):raise AdapterFault('AirSim did not respond')
            available=set(self.transport.call('listVehicles'))
            if not set(self.config.vehicle_ids)<=available:raise AdapterFault('configured vehicles missing')
            for n in self.config.vehicle_ids:
                self.original_api[n]=bool(self.transport.call('isApiControlEnabled',n))
                self.transport.call('enableApiControl',True,n)
                now=self.clock();brake=controls(brake=1)
                brake.update(command_timestamp_s=now,command_deadline_s=now+NATIVE_CONTROL_TIMEOUT_S,source_state_timestamp_s=now)
                self.transport.call('setCarControls',brake,n)
            now=self.clock();self.state_received={n:now for n in self.config.vehicle_ids}
            self.deadlines={n:now+self.config.command_ttl_s for n in self.config.vehicle_ids};self.opened=True
            self.read_states();self.record('OPEN',vehicles=list(self.config.vehicle_ids),origin_ue_m=self.config.origin_ue_m)
            if self.watchdog_enabled:
                self.thread=threading.Thread(target=self._watch,name='airsim-watchdog',daemon=True);self.thread.start()
            return self
        except Exception:
            self.close();raise

    def start_watchdog_after_staging(self,*,native_guard=False):
        """Arm once, before any drive command, after stationary fixture setup."""
        with self.lock:
            if not self.opened or self.closed or self.fault or self.thread is not None:
                raise AdapterFault('Invalid watchdog staging transition')
            if any(s>=0 for s in self.sequences.values()):
                raise AdapterFault('Cannot rearm watchdog after commands')
            states=self.read_states()
            if native_guard:
                row=self.snapshot_provider() if self.snapshot_provider else {}
                guards=row.get('states',{})
                if set(guards)!=set(self.config.vehicle_ids) or any(
                    r.get('native_guard_active') is not True or r.get('native_timed_controls_active') is not True
                    or r.get('native_control_timeout_s')!=NATIVE_CONTROL_TIMEOUT_S for r in guards.values()):
                    self.fail_stop('Native deadline guard not verified');raise AdapterFault(self.fault)
            if any(abs(s.speed)>.1 for s in states.values()):
                self.fail_stop('moving during watchdog staging')
                raise AdapterFault('Stationary staging required')
            now=self.clock()
            self.deadlines={n:now+self.config.command_ttl_s for n in self.config.vehicle_ids}
            self.watchdog_enabled=not native_guard
            if not native_guard:
                self.thread=threading.Thread(target=self._watch,name='airsim-watchdog',daemon=True)
                self.thread.start()
            self.record('STAGING_COMPLETE',native_deadline_guard=native_guard)

    def hold_for_replan(self):
        """Brake without clearing a fault or sending motion from old state."""
        with self.lock:
            if self.fault or self.closed or not self.opened:raise AdapterFault('Cannot hold a faulted or closed adapter')
            try:
                for n in self.original_api:self.transport.call('setCarControls',controls(brake=1),n)
            except Exception as exc:
                self.fail_stop('replan_brake_rpc: '+str(exc));raise AdapterFault(self.fault) from exc
            now=self.clock()
            for n in self.original_api:
                self.applied_speed[n]=0.;self.throttle[n]=0.;self.last_send[n]=now
                self.deadlines[n]=now+self.config.command_ttl_s
            self.record('REPLAN_HOLD')
            return now

    def _collision_info(self,name):
        hit=self.transport.call('simGetCollisionInfo',name)
        if hit.get('object_name_utf8_valid') is not False:return hit
        self.record('COLLISION_SNAPSHOT_INVALID',vehicle=name,collision=hit)
        # The native writer updates this structure without a synchronized
        # RPC snapshot. Recover a torn label only for the exact same event;
        # never replace an unknown impact with a newer ground contact.
        required={'time_stamp','has_collided','penetration_depth','normal',
                  'impact_point','position','object_id'}
        def identity(row):
            return {k:v for k,v in row.items()
                    if k not in ('object_name','object_name_utf8_valid')}
        if not required.issubset(hit) or hit['has_collided'] is not True or hit['time_stamp']<=0:
            raise AdapterFault('malformed native collision label '+name)
        recovered=[]
        for _ in range(2):
            row=self.transport.call('simGetCollisionInfo',name)
            if (row.get('object_name_utf8_valid') is False
                    or not isinstance(row.get('object_name'),str)
                    or identity(row)!=identity(hit)):
                raise AdapterFault('malformed native collision label '+name)
            recovered.append(row)
        if recovered[0]['object_name']!=recovered[1]['object_name']:
            raise AdapterFault('malformed native collision label '+name)
        self.record('COLLISION_SNAPSHOT_RECOVERED',vehicle=name,collision=recovered[-1])
        return recovered[-1]

    def read_states(self):
        if self.snapshot_provider is not None:return self._read_snapshot()
        try:
            if self.transport.call('simIsPaused'):raise AdapterFault('simulation paused')
            batch={};telemetry={};started=self.clock()
            for n in self.config.vehicle_ids:
                car=self.transport.call('getCarState',n)
                # CarState position is local to each spawn. ObjectPose uses the
                # current world origin and is the correct shared fleet frame.
                pose=self.transport.call('simGetObjectPose',n);p=pose['position'];q=pose['orientation']
                values=[float(p[k]) for k in ('x_val','y_val','z_val')]+[float(q[k]) for k in ('x_val','y_val','z_val','w_val')]
                finite(*values,float(car['speed']))
                qx,qy,qz,qw=values[3:];norm=math.sqrt(sum(v*v for v in values[3:]))
                if abs(norm-1)>.02:raise AdapterFault('invalid orientation '+n)
                heading=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
                stamp=int(car['timestamp']);received=self.clock()
                if stamp<=0 or stamp<self.last_source.get(n,stamp):raise AdapterFault('invalid source timestamp '+n)
                if stamp>self.last_source.get(n,-1):self.source_advanced[n]=received
                if received-self.source_advanced[n]>self.config.state_timeout_s:raise AdapterFault('source state frozen '+n)
                self.last_source[n]=stamp
                ox,oy,oz=self.config.origin_ue_m
                batch[n]=VehicleState(n,values[0]+ox,values[1]+oy,heading,speed=float(car['speed']),timestamp=received)
                telemetry[n]={'ue_z_m':oz-values[2],'source_timestamp_ns':stamp,
                              'collision':self._collision_info(n)}
                if telemetry[n]['collision'].get('object_name_utf8_valid') is False:
                    raise AdapterFault('malformed native collision label '+n)
            if self.clock()-started>self.config.state_timeout_s:raise AdapterFault('state batch timed out')
            with self.lock:
                self.states=batch;self.telemetry=telemetry
                for n in batch:self.state_received[n]=batch[n].timestamp
            self.record('STATE',vehicles={n:dict(x=s.x,y=s.y,heading=s.heading,speed=s.speed,timestamp=s.timestamp,**telemetry[n]) for n,s in batch.items()})
            return batch
        except Exception as e:
            self.fail_stop('state_read: '+str(e));raise AdapterFault(str(e)) from e

    def _read_snapshot(self):
        try:
            row=self.snapshot_provider();now=self.clock();stamp=row['sequence'];received=row['monotonic_s']
            if row['paused']:raise AdapterFault('simulation paused')
            if set(row['states'])!=set(self.config.vehicle_ids):raise AdapterFault('incomplete fleet snapshot')
            if not 0<=now-received<=self.config.state_timeout_s:raise AdapterFault('stale fleet snapshot')
            batch={};telemetry={}
            for n,r in row['states'].items():
                finite(r['x'],r['y'],r['z'],r['heading'],r['speed'])
                if stamp<self.last_source.get(n,stamp):raise AdapterFault('out-of-order fleet snapshot')
                if stamp>self.last_source.get(n,-1):self.source_advanced[n]=received
                if now-self.source_advanced[n]>self.config.state_timeout_s:raise AdapterFault('source state frozen '+n)
                self.last_source[n]=stamp
                batch[n]=VehicleState(n,r['x'],r['y'],r['heading'],speed=r['speed'],timestamp=received)
                telemetry[n]=dict(ue_z_m=r['z'],source_timestamp_ns=stamp,collision=self._collision_info(n))
                if telemetry[n]['collision'].get('object_name_utf8_valid') is False:
                    raise AdapterFault('malformed native collision label '+n)
            with self.lock:
                self.states=batch;self.telemetry=telemetry
                for n in batch:self.state_received[n]=received
            self.record('STATE',vehicles={n:dict(x=s.x,y=s.y,heading=s.heading,speed=s.speed,timestamp=s.timestamp,**telemetry[n]) for n,s in batch.items()})
            return batch
        except Exception as exc:
            self.fail_stop('fleet_snapshot: '+str(exc));raise AdapterFault(str(exc)) from exc

    def send(self,command: ControlCommand):
        n=command.vehicle_id
        with self.lock:
            now=self.clock()
            if not self.opened or self.closed or n not in self.sequences:raise AdapterFault('adapter closed or vehicle unknown')
            if self.fault:raise AdapterFault('latched: '+self.fault)
            if not command.is_valid_at(now,self.sequences[n]):
                self.fail_stop('invalid, expired or out-of-order command '+n);raise AdapterFault(self.fault)
            if now-self.state_received.get(n,0)>self.config.state_timeout_s:
                self.fail_stop('state_timeout '+n);raise AdapterFault(self.fault)
            if (command.v_target<0 and not self.config.allow_reverse) or (abs(command.v_target)<.05 and abs(command.omega_target)>1e-6):
                self.fail_stop('unsupported reverse or stationary rotation');raise AdapterFault(self.fault)
            if command.steering_rad is not None and abs(command.steering_rad)>self.config.max_steer_rad:
                self.fail_stop('steering angle exceeds configured limit');raise AdapterFault(self.fault)
            dt=min(.3,max(.01,now-self.last_send[n]));self.last_send[n]=now
            requested_direction=-1 if command.v_target<0 else 1
            changing=abs(command.v_target)>=.05 and requested_direction!=self.drive_direction[n]
            if changing and abs(self.states[n].speed)<.08:
                self.drive_direction[n]=requested_direction;self.applied_speed[n]=0.;self.throttle[n]=0.
                changing=False
            target=0. if changing else min(abs(command.v_target),self.applied_speed[n]+self.config.speed_rise_m_s2*dt)
            self.applied_speed[n]=target
            error=target-abs(self.states[n].speed)
            throttle=0 if target<MIN_ACTUATED_SPEED_M_S or error<-.1 else min(self.config.max_throttle,max(0,.12+self.config.speed_error_gain*error))
            throttle=min(throttle,self.throttle[n]+self.config.throttle_rise_per_s*dt);self.throttle[n]=throttle
            brake=1 if target<MIN_ACTUATED_SPEED_M_S else min(.4,max(0,-.3*(error+.1)))
            if brake>0:throttle=0.;self.throttle[n]=0.
            # Compute steering from the requested yaw rate / speed so target
            # speed smoothing does not change the path's curvature.
            divisor=math.copysign(max(.05,abs(command.v_target)),command.v_target)
            steer=math.atan(self.config.wheelbase_m*command.omega_target/divisor)/self.config.max_steer_rad
            if command.steering_rad is not None:
                # Keep the requested wheel angle during braking/coasting.
                # Zero target speed still applies full brake and no throttle.
                steer=command.steering_rad/self.config.max_steer_rad
            native=controls(throttle,brake,max(-1,min(1,steer)))
            native.update(command_timestamp_s=now,
                          command_deadline_s=min(command.timestamp+command.ttl,now+NATIVE_CONTROL_TIMEOUT_S),
                          source_state_timestamp_s=self.state_received[n])
            if self.config.allow_reverse:
                native.update(is_manual_gear=True,manual_gear=self.drive_direction[n])
            try:self.transport.call('setCarControls',native,n)
            except Exception as e:
                self.fail_stop('control_rpc: '+str(e));raise AdapterFault(str(e)) from e
            self.sequences[n]=command.sequence_id;self.deadlines[n]=min(command.timestamp+command.ttl,now+self.config.command_ttl_s)
            self.record('COMMAND',vehicle=n,sequence=command.sequence_id,v_target=command.v_target,
                        omega_target=command.omega_target,steering_rad=command.steering_rad,deadline=self.deadlines[n],native=native)

    def fail_stop(self,reason,transport=None):
        with self.lock:
            if not self.fault:self.fault=reason;self.record('FAULT',reason=reason)
            rpc=transport or self.transport
            for n in self.original_api:
                try:rpc.call('setCarControls',controls(brake=1),n)
                except Exception as e:self.record('BRAKE_RPC_FAILED',vehicle=n,error=str(e))
            self.record('BRAKE',reason=reason)

    def check_deadlines(self,transport=None):
        with self.lock:
            if self.closed or not self.opened or self.fault:return
            now=self.clock()
            reason=next(('state_timeout '+n for n in self.config.vehicle_ids if now-self.state_received[n]>self.config.state_timeout_s),None)
            if reason is None:reason=next(('command_timeout '+n for n in self.config.vehicle_ids if now>=self.deadlines[n]),None)
            if reason:self.fail_stop(reason,transport)

    def _watch(self):
        rpc=None
        try:
            rpc=self.factory()  # Independent connection and thread-owned I/O loop.
            while not self.shutdown.wait(.05):self.check_deadlines(rpc)
        except Exception as e:
            if rpc is not None:self.fail_stop('watchdog: '+str(e),rpc)
            else:
                with self.lock:self.fault='watchdog: '+str(e);self.record('FAULT',reason=self.fault)
        finally:
            if rpc is not None:rpc.close()

    def close(self):
        if self.closed:return
        self.shutdown.set()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=self.config.rpc_timeout_s*len(self.config.vehicle_ids)+1)
        # Keep ownership during braking; releasing API before stopping clears
        # the native control inputs and can leave a moving vehicle coasting.
        errors=[]
        for n in self.original_api:
            try:self.transport.call('setCarControls',controls(brake=1),n)
            except Exception as e:errors.append(str(e));self.record('CLOSE_BRAKE_FAILED',vehicle=n,error=str(e))
        stopped=False;end=time.monotonic()+2
        while self.original_api and time.monotonic()<end:
            try:
                stopped=all(abs(self.transport.call('getCarState',n)['speed'])<.01 for n in self.original_api)
                if stopped:break
            except Exception:break
            time.sleep(.05)
        for n,old in self.original_api.items():
            if stopped:
                try:self.transport.call('enableApiControl',old,n)
                except Exception as e:errors.append(str(e));self.record('API_RESTORE_FAILED',vehicle=n,error=str(e))
        self.close_result={'stopped':stopped,'api_restored':stopped and not errors,'errors':errors}
        self.record('CLOSE',**self.close_result)
        self.closed=True;self.transport.close();self.log.close()

    def __enter__(self):return self.open()
    def __exit__(self,*args):self.close()
