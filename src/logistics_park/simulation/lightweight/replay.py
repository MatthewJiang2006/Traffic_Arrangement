"""Standalone offline replay, without network assets or runtime dependencies."""
import json
from pathlib import Path


def write_replay(simulator, output: Path) -> None:
    frames=[]
    for row in simulator.rows:
        if not frames or frames[-1]['t']!=row['timestamp']:
            frames.append({'t':row['timestamp'],'vehicles':[]})
        frames[-1]['vehicles'].append({
            'id':row['vehicle_id'],'x':row['x'],'y':row['y'],'heading':row['heading'],
            'speed':row['speed'],'status':row['task_status'],
            'owner':row['owned_intersections'].split(',') if row['owned_intersections'] else []})
    payload={'frames':frames,'vehicle':{'length':simulator.config.vehicle.length,'width':simulator.config.vehicle.width},
             'roads':[{'points':edge.centerline.points,'width':edge.width} for edge in simulator.graph.edges.values()],
             'zones':[{'id':spec.intersection_id,'bounds':spec.bounds} for spec in simulator.intersections.values()]}
    # Escaping '<' prevents user-provided names from closing the embedded script.
    encoded=json.dumps(payload,separators=(',',':')).replace('<','\\u003c')
    template=r'''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FCFS 多车验证回放</title>
<style>
body{font:16px system-ui,sans-serif;color:#172033;background:#f8fafc;max-width:1100px;margin:24px auto;padding:0 18px}
h1{font-size:24px}p{color:#475569}button,select{font:inherit;padding:6px 14px}input{flex:1;min-width:120px}
.controls{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin:18px 0}canvas{width:100%;background:white;border:1px solid #cbd5e1;border-radius:6px}
#info{white-space:pre-line;font-size:14px;line-height:1.8}button:focus-visible,input:focus-visible,select:focus-visible{outline:3px solid #2563eb}
</style>
<h1>FCFS 多车验证回放</h1>
<p>临时测试地图。拖动时间条查看车辆位置；绿色区域表示路口已有车辆获准通行。</p>
<div class="controls"><button id="play">播放</button><button id="reset">回到开始</button>
<label>速度 <select id="rate"><option value="1">1 倍</option><option value="5" selected>5 倍</option><option value="10">10 倍</option></select></label>
<input id="timeline" type="range" min="0" value="0" aria-label="仿真时间"><span id="time"></span></div>
<canvas id="map" width="1100" height="680" aria-label="车辆与路口回放地图"></canvas><p id="info"></p>
<script id="replay-data" type="application/json">__DATA__</script>
<script>
const data=JSON.parse(document.getElementById('replay-data').textContent);
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d');
const slider=document.getElementById('timeline'),play=document.getElementById('play');
const colors=['#2563eb','#dc2626','#059669','#9333ea','#d97706','#0891b2'];
const ids=[...new Set(data.frames.flatMap(f=>f.vehicles.map(v=>v.id)))].sort();
const points=data.roads.flatMap(r=>r.points).concat(data.frames.flatMap(f=>f.vehicles.map(v=>[v.x,v.y])));
const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
const minX=Math.min(...xs)-.18,maxX=Math.max(...xs)+.18,minY=Math.min(...ys)-.18,maxY=Math.max(...ys)+.18;
const scale=Math.min((canvas.width-64)/(maxX-minX),(canvas.height-64)/(maxY-minY));
const xy=(x,y)=>[32+(x-minX)*scale,canvas.height-32-(y-minY)*scale];
slider.max=data.frames.length-1;
let index=0,playing=false,last=null,simTime=0;
function draw(){
  const frame=data.frames[index];ctx.clearRect(0,0,canvas.width,canvas.height);
  for(const road of data.roads){ctx.beginPath();road.points.forEach((p,i)=>{const q=xy(...p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.lineWidth=road.width*scale;ctx.strokeStyle='#e2e8f0';ctx.stroke()}
  for(const zone of data.zones){const owner=frame.vehicles.find(v=>v.owner.includes(zone.id));const b=zone.bounds;const q=xy(b[0],b[3]);
    ctx.fillStyle=owner?'#bbf7d0':'#fef3c7';ctx.fillRect(q[0],q[1],(b[2]-b[0])*scale,(b[3]-b[1])*scale);
    ctx.strokeStyle='#64748b';ctx.lineWidth=1;ctx.strokeRect(q[0],q[1],(b[2]-b[0])*scale,(b[3]-b[1])*scale);
    ctx.fillStyle='#334155';ctx.font='14px sans-serif';ctx.fillText(zone.id+(owner?' · '+owner.id:''),q[0]+6,q[1]+18)}
  for(const road of data.roads){ctx.beginPath();road.points.forEach((p,i)=>{const q=xy(...p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.lineWidth=1;ctx.setLineDash([5,5]);ctx.strokeStyle='#94a3b8';ctx.stroke();ctx.setLineDash([])}
  for(const v of frame.vehicles){const q=xy(v.x,v.y);ctx.save();ctx.translate(...q);ctx.rotate(-v.heading);ctx.fillStyle=colors[ids.indexOf(v.id)%colors.length];
    const l=data.vehicle.length*scale,w=data.vehicle.width*scale;ctx.fillRect(-l/2,-w/2,l,w);ctx.fillStyle='white';ctx.beginPath();ctx.moveTo(l*.35,0);ctx.lineTo(0,-w*.25);ctx.lineTo(0,w*.25);ctx.fill();ctx.restore();
    ctx.fillStyle='#172033';ctx.font='13px sans-serif';ctx.fillText(v.id,q[0]+10,q[1]-10)}
  slider.value=index;document.getElementById('time').textContent=frame.t.toFixed(2)+' s';
  const status={EN_ROUTE:'运输中',ARRIVED:'等待装卸',SERVICING:'装卸中',IDLE:'停靠'};
  document.getElementById('info').textContent=frame.vehicles.map(v=>v.id+'：'+(status[v.status]||v.status)+' · '+v.speed.toFixed(2)+' m/s'+(v.owner.length?' · 持有路口 '+v.owner.join(', '):'')).join('\n');
}
function setPlaying(value){playing=value;last=null;play.textContent=playing?'暂停':'播放'}
play.addEventListener('click',()=>{if(index===data.frames.length-1){index=0;simTime=0}setPlaying(!playing);draw()});
document.getElementById('reset').addEventListener('click',()=>{setPlaying(false);index=0;simTime=0;draw()});
slider.addEventListener('input',()=>{setPlaying(false);index=Number(slider.value);simTime=data.frames[index].t;draw()});
function tick(stamp){if(playing){if(last!==null)simTime+=(stamp-last)/1000*Number(document.getElementById('rate').value);
  while(index+1<data.frames.length&&data.frames[index+1].t<=simTime)index++;
  if(index===data.frames.length-1)setPlaying(false);draw();last=stamp}requestAnimationFrame(tick)}
draw();requestAnimationFrame(tick);
</script></html>'''
    output.write_text(template.replace('__DATA__',encoded),encoding='utf-8')
