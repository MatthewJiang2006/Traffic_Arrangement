"""Curvature-bounded rear-axle paths for four terminal U-turn fixtures."""
import math
TAU=2*math.pi
WHEELBASE=2.3285

def line(a,b,step=.25):
 n=max(1,math.ceil(math.dist(a,b)/step));return [[a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n] for i in range(n+1)]

def arc(center,start,end,sign,r):
 a=math.atan2(start[1]-center[1],start[0]-center[0]);b=math.atan2(end[1]-center[1],end[0]-center[0]);sweep=(sign*(b-a))%TAU
 n=max(1,math.ceil(sweep*r/.20));return [[center[0]+r*math.cos(a+sign*sweep*i/n),center[1]+r*math.sin(a+sign*sweep*i/n)] for i in range(n+1)]

def ccc(a,b,yaw,r,sign,side):
 endyaw=yaw+math.pi
 ca=[a[0]-sign*r*math.sin(yaw),a[1]+sign*r*math.cos(yaw)]
 cb=[b[0]-sign*r*math.sin(endyaw),b[1]+sign*r*math.cos(endyaw)]
 dx,dy=cb[0]-ca[0],cb[1]-ca[1];d=math.hypot(dx,dy)
 if not 0<d<4*r:return None
 h=math.sqrt(4*r*r-d*d/4);mid=[(ca[0]+cb[0])/2-side*h*dy/d,(ca[1]+cb[1])/2+side*h*dx/d]
 p=[(ca[0]+mid[0])/2,(ca[1]+mid[1])/2];q=[(cb[0]+mid[0])/2,(cb[1]+mid[1])/2]
 return arc(ca,a,p,sign,r)[:-1]+arc(mid,p,q,-sign,r)[:-1]+arc(cb,q,b,sign,r)

def make_paths(survey):
 out={}
 for label,junction in [('A','j_0_0'),('B','j_1_2'),('C','j_3_0'),('D','j_3_2')]:
  cx,cy=survey['junctions'][junction]['center_m']
  if label in ('A','C'):
   sign=1 if label=='A' else -1;begin_y=65 if label=='A' else 289
   incoming_x=cx-sign*5.7;inner_x=cx-sign*1.9;outgoing_x=cx+sign*5.7
   rear_begin=begin_y-sign*WHEELBASE/2
   approach=[]
   # Smooth change from right to left lane, finished before the junction.
   length=14;shift_end=rear_begin+sign*length
   for i in range(81):
    t=i/80;s=10*t**3-15*t**4+6*t**5
    approach.append([incoming_x+(inner_x-incoming_x)*s,rear_begin+sign*length*t])
   turn_y=cy-sign*4.0
   start=[inner_x,turn_y];end=[outgoing_x,turn_y]
   center=[cx+sign*1.9,turn_y]
   path=approach[:-1]+line(approach[-1],start)[:-1]+arc(center,start,end,-1,3.8)[:-1]+line(end,[outgoing_x,begin_y+sign*WHEELBASE/2])
   out[label]=[{'rear_path':path,'start_yaw_deg':90*sign,'radius_m':3.8,'mode':'right-to-left lane change then left semicircle to opposite right lane','junction':junction}]
  else:
   candidates=[]
   # Turn right-left-right or left-right-left within the empty junction.
   # Each arc obeys the truck's minimum radius; no reverse or teleport.
   for r in (3.2,3.5,3.8,4.1,4.4):
    for offset in (-8,-7,-6,-5,-4,-3):
     a=[cx+offset,cy+1.9];b=[cx+offset,cy-1.9]
     for sign in (1,-1):
      for side in (1,-1):
       turn=ccc(a,b,0,r,sign,side)
       if turn is None:continue
       # Reserve room for the forward cab and trailing short cargo.
       if max(p[0] for p in turn)>cx+5.5 or min(p[0] for p in turn)<cx-8:continue
       if max(abs(p[1]-cy) for p in turn)>6:continue
       path=line([23-WHEELBASE/2,cy+1.9],a)[:-1]+turn[:-1]+line(b,[23+WHEELBASE/2,cy-1.9])
       length=sum(math.dist(x,y) for x,y in zip(path,path[1:]))
       candidates.append({'rear_path':path,'start_yaw_deg':0,'radius_m':r,'mode':'forward curvature-bounded three-arc U-turn','junction':junction,'length_m':length})
   out[label]=sorted(candidates,key=lambda p:p['length_m'])
 return out
