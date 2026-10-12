"""Accelerated continuous transport on the measured UE park geometry."""
import argparse
import csv
import json
import math
import time
from pathlib import Path
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import TransportController


def simulate(model,vehicle_count=8,cycles=2,max_time=4000.,dt=.02,outlet_block_until=None,shared_resources=False):
    names=['Car_0_0','Car_0_1','Car_0_2','Car_1_0','Car_1_1','Car_1_2','Car_2_0','Car_2_1'][:vehicle_count]
    pairs=[('B','A'),('D','C'),('A','D'),('C','B')]*2
    plans={n:model.plan(a,[b,a]*cycles) for n,(a,b) in zip(names,pairs)}
    controller=TransportController(model,plans,outlet_block_until=outlet_block_until,shared_resources=shared_resources);states={};pending=list(names)
    releases={n:(i%4)*15+(i//4)*120 for i,n in enumerate(names)}
    commands={};frames=[];started=time.perf_counter();failure=[];ticks=math.ceil(max_time/dt)
    for tick in range(ticks):
        now=tick*dt
        if tick%5==0:
            for n in list(pending):
                if now>=releases[n] and controller.can_activate(n):
                    plan=plans[n];p=plan.point(0);states[n]=VehicleState(n,*p,plan.heading(0),timestamp=now)
                    controller.activate(n,now);pending.remove(n)
            try:commands=controller.tick(states,now)
            except RuntimeError as exc:failure.append(str(exc));break
            for n,ex in controller.executions.items():
                if ex.finished:states.pop(n,None)
            if tick%25==0:
                frames.append({'t':now,'vehicles':[{'id':n,'x':s.x,'y':s.y,'heading':s.heading,'speed':s.speed,
                    'status':'SERVICING' if controller.executions[n].service_until is not None else 'EN_ROUTE',
                    'owner':[j for j,g in controller.scheduler.owners.items() if g.request.vehicle_id==n]} for n,s in states.items()]})
            if not pending and all(ex.finished for ex in controller.executions.values()):break
        for n,state in states.items():
            target,delta=commands.get(n,(0.,0.))
            dv=max(-.6*dt,min(.5*dt,target-state.speed));speed=max(0,state.speed+dv)
            middle_heading=state.heading+.5*speed*math.tan(delta)/2.3285*dt
            state.x+=speed*math.cos(middle_heading)*dt;state.y+=speed*math.sin(middle_heading)*dt
            state.heading=(state.heading+speed*math.tan(delta)/2.3285*dt+math.pi)%(2*math.pi)-math.pi
            state.speed=speed;state.timestamp=now+dt
    else:failure.append('simulation timeout')
    report=controller.report();report.update(failure=failure,execution='accelerated_bicycle_kinematics',
        ue_physics_validated=False,service_mode='2-second virtual road-test service; not a warehouse berth',
        wall_time_s=time.perf_counter()-started,dt_s=dt,control_hz=10,cycles_per_vehicle=cycles)
    report['acceleration_factor']=report['simulation_time_s']/max(.001,report['wall_time_s'])
    if failure:report['status']='FAIL'
    return report,frames,plans


def write_results(model,report,frames,plans,output):
    output.mkdir(parents=True,exist_ok=True)
    (output/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'routes.json').write_text(json.dumps({n:{'origin':p.origin,'stops':p.stops,'edges':p.edges,
        'length_m':p.length,'checkpoints':p.checkpoints} for n,p in plans.items()},indent=2)+'\n')
    with (output/'trajectory.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','vehicle_id','rear_x_m','rear_y_m','heading_rad','speed_m_s'])
        for frame in frames:
            for v in frame['vehicles']:writer.writerow([frame['t'],v['id'],v['x'],v['y'],v['heading'],v['speed']])
    write_replay(model,frames,output/'replay.html')


def write_replay(model,frames,path):
    # Offline artifact, all data embedded; the UE viewport remains untouched.
    payload={'frames':frames,'roads':[{'points':e['centerline'],'width':e['width']} for e in model.edges.values()],
             'zones':[{'id':k,'bounds':v['bounds']} for k,v in model.zones.items()],
             'sites':{k:v['start']['xy_m'] for k,v in model.sites.items()}}
    encoded=json.dumps(payload,separators=(',',':')).replace('<','\\u003c')
    html=r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>园区连续运输轻量仿真</title>
<style>body{font:16px system-ui;margin:24px;background:#f8fafc;color:#172033}canvas{background:white;border:1px solid #cbd5e1;max-width:100%}input{width:50%}button,select{font:inherit;padding:6px}#info{white-space:pre-line}</style>
<h1>园区连续运输轻量仿真</h1><p>使用 UE 实测道路拓扑。此回放展示轻量运动学结果，接地和静态场景碰撞需在 UE 验证。</p>
<button id="play">播放</button> <select id="rate"><option>1</option><option>5</option><option selected>20</option><option>50</option></select> 倍
<input id="slider" type="range" min="0" value="0"><span id="time"></span><p><canvas id="canvas" width="1000" height="900"></canvas></p><p id="info"></p>
<script id="data" type="application/json">DATA</script><script>
const data=JSON.parse(document.querySelector('#data').textContent),c=document.querySelector('canvas'),ctx=c.getContext('2d'),slider=document.querySelector('#slider');
const points=data.roads.flatMap(r=>r.points),xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
const xmin=Math.min(...xs)-8,xmax=Math.max(...xs)+8,ymin=Math.min(...ys)-8,ymax=Math.max(...ys)+8,scale=Math.min(930/(xmax-xmin),830/(ymax-ymin));
const xy=(x,y)=>[35+(x-xmin)*scale,865-(y-ymin)*scale],colors=['#2563eb','#dc2626','#059669','#9333ea','#d97706','#0891b2','#be185d','#475569'];
const names=[...new Set(data.frames.flatMap(f=>f.vehicles.map(v=>v.id)))];let i=0,playing=false,last=null,t=0;slider.max=Math.max(0,data.frames.length-1);
function draw(){if(!data.frames.length)return;let f=data.frames[i];ctx.clearRect(0,0,c.width,c.height);
for(let r of data.roads){ctx.beginPath();r.points.forEach((p,j)=>{let q=xy(...p);j?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.lineWidth=r.width*scale;ctx.strokeStyle='#e2e8f0';ctx.stroke()}
for(let z of data.zones){let [x0,y0,x1,y1]=z.bounds,q=xy(x0,y1);ctx.fillStyle=f.vehicles.some(v=>v.owner.includes(z.id))?'#bbf7d080':'#fde68a50';ctx.fillRect(...q,(x1-x0)*scale,(y1-y0)*scale)}
for(let [id,p] of Object.entries(data.sites)){let q=xy(...p);ctx.fillStyle='#111827';ctx.font='bold 18px sans-serif';ctx.fillText(id,...q)}
for(let v of f.vehicles){let q=xy(v.x+.75*Math.cos(v.heading),v.y+.75*Math.sin(v.heading));ctx.save();ctx.translate(...q);ctx.rotate(-v.heading);ctx.fillStyle=colors[names.indexOf(v.id)%8];ctx.fillRect(-4.172*scale,-1.3*scale,8.344*scale,2.6*scale);ctx.fillStyle='white';ctx.fillRect(2.6*scale,-.8*scale,.8*scale,1.6*scale);ctx.restore();ctx.fillStyle='#0f172a';ctx.font='12px sans-serif';ctx.fillText(v.id,q[0]+8,q[1]-8)}
slider.value=i;document.querySelector('#time').textContent=f.t.toFixed(1)+' s';document.querySelector('#info').textContent=f.vehicles.map(v=>v.id+' · '+v.speed.toFixed(2)+' m/s · '+(v.status==='SERVICING'?'测试点服务':'运输中')).join('\n')}
document.querySelector('#play').onclick=()=>{if(i===data.frames.length-1){i=0;t=0}playing=!playing;last=null;document.querySelector('#play').textContent=playing?'暂停':'播放'};
slider.oninput=()=>{i=+slider.value;t=data.frames[i].t;playing=false;document.querySelector('#play').textContent='播放';draw()};
function tick(now){if(playing){if(last!==null)t+=(now-last)/1000*+document.querySelector('#rate').value;while(i+1<data.frames.length&&data.frames[i+1].t<=t)i++;if(i===data.frames.length-1)playing=false;draw();last=now}requestAnimationFrame(tick)}draw();requestAnimationFrame(tick);
</script></html>'''
    path.write_text(html.replace('>DATA<','>'+encoded+'<'),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,default=Path('config/topology/industrialarea_ue_transport.json'))
    parser.add_argument('--vehicles',type=int,choices=(1,2,4,8),default=8)
    parser.add_argument('--cycles',type=int,default=2)
    parser.add_argument('--max-time',type=float,default=4000)
    parser.add_argument('--block-until',type=float,default=0.,help='test fixture: prohibit terminal admission until this simulation time')
    parser.add_argument('--block-resource',default='j_3_2')
    parser.add_argument('--shared-resources',action='store_true',help='enable FIFO shared-channel reservations')
    parser.add_argument('--output',type=Path,default=Path('runs/ue_transport_lightweight'))
    args=parser.parse_args()
    if args.cycles<1:parser.error('cycles must be positive')
    if not math.isfinite(args.block_until) or args.block_until<0:parser.error('block time must be finite and nonnegative')
    block={args.block_resource:args.block_until} if args.block_until else None
    model=TransportMap(args.map);report,frames,plans=simulate(model,args.vehicles,args.cycles,args.max_time,outlet_block_until=block,shared_resources=args.shared_resources)
    write_results(model,report,frames,plans,args.output)
    print(json.dumps({k:v for k,v in report.items() if k not in ('task_events','traffic_events')},indent=2),flush=True)
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
