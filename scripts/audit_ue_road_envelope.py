"""Check complete truck envelopes on straight route spans in a recorded UE run."""
import json,math,bisect,pathlib,argparse
root=pathlib.Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('run',type=pathlib.Path);parser.add_argument('--output',type=pathlib.Path,required=True)
args=parser.parse_args();demo=args.run.resolve();edges={e['edge_id']:e for e in json.load(open(root/'config/topology/industrialarea_ue_internal_waiting_transport.json'))['edges']};timeline=[];times=[];cache={}
for line in open(demo/'physical_control_diagnostics.jsonl'):
 row=json.loads(line);specs={}
 for name,d in row['execution_details'].items():
  key=d['edge_id'];e=edges.get(key)
  if not e or e['kind']!='road':continue
  route=tuple(d['route_edges']);tag=(route,key)
  if tag not in cache:
   i=route.index(key);a=b=i
   while a>0 and edges.get(route[a-1],{}).get('capacity_alias')==e['capacity_alias'] and edges[route[a-1]]['kind']=='road':a-=1
   while b<len(route)-1 and edges.get(route[b+1],{}).get('capacity_alias')==e['capacity_alias'] and edges[route[b+1]]['kind']=='road':b+=1
   p=edges[route[a]]['centerline'][0];q=edges[route[b]]['centerline'][-1];length=math.dist(p,q);co=(q[0]-p[0])/length;si=(q[1]-p[1])/length
   cache[tag]=(p,q,length,co,si,e['width'],e['capacity_alias'])
  specs[name]=cache[tag]
 times.append(row['t_s']);timeline.append(specs)
flags=[];samples=0;maximum=-999
for line in open(demo/'physical_trajectory.jsonl'):
 f=json.loads(line);i=bisect.bisect_right(times,f['t_s'])-1
 if i<0 or f['t_s']-times[i]>1.5:continue
 for name,s in f['positions'].items():
  spec=timeline[i].get(name)
  if spec is None:continue
  p,q,length,co,si,width,alias=spec;x,y,h,v=s;cx=x+.75*math.cos(h);cy=y+.75*math.sin(h);diff=h-math.atan2(si,co)
  along=4.172*abs(math.cos(diff))+1.3*abs(math.sin(diff));side=4.172*abs(math.sin(diff))+1.3*abs(math.cos(diff));long=(cx-p[0])*co+(cy-p[1])*si;lat=abs(-(cx-p[0])*si+(cy-p[1])*co)
  if long<along+.25 or long>length-along-.25:continue
  samples+=1;excess=lat+side-width/2;maximum=max(maximum,excess)
  if excess>.05:flags.append(dict(t_s=f['t_s'],vehicle=name,alias=alias,excess_m=excess,heading_deg=math.degrees(diff)))
r=dict(complete_body_road_samples=samples,maximum_envelope_excess_m=maximum,flags=flags,scope='Straight route road spans only; complete body stays between entry and exit attachment boundaries; geometry from latest preceding diagnostic <=1.5 s.')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(r,indent=2));print('samples',samples,'flags',len(flags),'maximum',maximum);print(flags[:10])
