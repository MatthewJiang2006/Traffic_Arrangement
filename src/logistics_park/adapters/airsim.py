"""AirSim car RPC backend, UE world coordinates, deadlines and fail-stop ownership."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable, Protocol
from logistics_park.common.messages import ControlCommand, VehicleState, finite


class Transport(Protocol):
    def call(self, method: str, *args): ...
    def close(self): ...


class RPCTransport:
    def __init__(self, host: str, port: int, timeout: float):
        import msgpackrpc  # Optional backend dependency; lightweight simulation needs none.
        self.client = msgpackrpc.Client(msgpackrpc.Address(host, port), timeout=timeout,
                                        pack_encoding='utf-8', unpack_encoding='utf-8')
    def call(self, method, *args):
        return self.client.call(method, *args)
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
                self.transport.call('setCarControls',controls(brake=1),n)
            now=self.clock();self.state_received={n:now for n in self.config.vehicle_ids}
            self.deadlines={n:now+self.config.command_ttl_s for n in self.config.vehicle_ids};self.opened=True
            self.read_states();self.record('OPEN',vehicles=list(self.config.vehicle_ids),origin_ue_m=self.config.origin_ue_m)
            if self.watchdog_enabled:
                self.thread=threading.Thread(target=self._watch,name='airsim-watchdog',daemon=True);self.thread.start()
            return self
        except Exception:
            self.close();raise

    def start_watchdog_after_staging(self):
        """Arm once, before any drive command, after stationary fixture setup."""
        with self.lock:
            if not self.opened or self.closed or self.fault or self.thread is not None:
                raise AdapterFault('Invalid watchdog staging transition')
            if any(s>=0 for s in self.sequences.values()):
                raise AdapterFault('Cannot rearm watchdog after commands')
            states=self.read_states()
            if any(abs(s.speed)>.1 for s in states.values()):
                self.fail_stop('moving during watchdog staging')
                raise AdapterFault('Stationary staging required')
            now=self.clock()
            self.deadlines={n:now+self.config.command_ttl_s for n in self.config.vehicle_ids}
            self.watchdog_enabled=True
            self.thread=threading.Thread(target=self._watch,name='airsim-watchdog',daemon=True)
            self.thread.start()
            self.record('STAGING_COMPLETE')

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
                              'collision':self.transport.call('simGetCollisionInfo',n)}
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
                telemetry[n]=dict(ue_z_m=r['z'],source_timestamp_ns=stamp,collision=self.transport.call('simGetCollisionInfo',n))
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
            dt=min(.3,max(.01,now-self.last_send[n]));self.last_send[n]=now
            requested_direction=-1 if command.v_target<0 else 1
            changing=abs(command.v_target)>=.05 and requested_direction!=self.drive_direction[n]
            if changing and abs(self.states[n].speed)<.08:
                self.drive_direction[n]=requested_direction;self.applied_speed[n]=0.;self.throttle[n]=0.
                changing=False
            target=0. if changing else min(abs(command.v_target),self.applied_speed[n]+self.config.speed_rise_m_s2*dt)
            self.applied_speed[n]=target
            error=target-abs(self.states[n].speed)
            throttle=0 if target<.08 or error<-.1 else min(self.config.max_throttle,max(0,.12+self.config.speed_error_gain*error))
            throttle=min(throttle,self.throttle[n]+self.config.throttle_rise_per_s*dt);self.throttle[n]=throttle
            brake=1 if target<.08 else min(.4,max(0,-.3*(error+.1)))
            if brake>0:throttle=0.;self.throttle[n]=0.
            # Compute steering from the requested yaw rate / speed so target
            # speed smoothing does not change the path's curvature.
            divisor=math.copysign(max(.05,abs(command.v_target)),command.v_target)
            steer=math.atan(self.config.wheelbase_m*command.omega_target/divisor)/self.config.max_steer_rad
            native=controls(throttle,brake,max(-1,min(1,steer)))
            if self.config.allow_reverse:
                native.update(is_manual_gear=True,manual_gear=self.drive_direction[n])
            try:self.transport.call('setCarControls',native,n)
            except Exception as e:
                self.fail_stop('control_rpc: '+str(e));raise AdapterFault(str(e)) from e
            self.sequences[n]=command.sequence_id;self.deadlines[n]=min(command.timestamp+command.ttl,now+self.config.command_ttl_s)
            self.record('COMMAND',vehicle=n,sequence=command.sequence_id,v_target=command.v_target,
                        omega_target=command.omega_target,deadline=self.deadlines[n],native=native)

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
