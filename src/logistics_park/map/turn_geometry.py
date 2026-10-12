"""Truck rear-axle quarter turns with delayed yaw to clear the inner curb."""
import math

def cardinal_axes(points):
    def axis(a,b):
        yaw=round(math.atan2(b[1]-a[1],b[0]-a[0])/(math.pi/2))*(math.pi/2)
        return (round(math.cos(yaw)),round(math.sin(yaw)))
    return axis(points[0],points[1]),axis(points[-2],points[-1])

def right_turn_path(edge,radius=None):
    if radius is None:radius=3.2 if edge.get('incoming_lane')=='RIGHT' else 4.
    a,b=edge['centerline'][0],edge['centerline'][-1]
    incoming,outgoing=cardinal_axes(edge['centerline'])
    if incoming[0]*outgoing[1]-incoming[1]*outgoing[0]!=1:
        raise ValueError('UE right turn must have positive yaw')
    distances=[sum((b[i]-a[i])*direction[i] for i in (0,1)) for direction in (incoming,outgoing)]
    if min(distances)<=radius or radius<2.3285/math.tan(math.radians(40)):
        raise ValueError('turn geometry outside truck steering limits')
    start=[a[i]+(distances[0]-radius)*incoming[i] for i in (0,1)]
    centre=[start[i]+radius*outgoing[i] for i in (0,1)]
    points=[a,start]
    for j in range(1,61):
        t=math.pi/2*j/60
        points.append([centre[i]-radius*outgoing[i]*math.cos(t)+radius*incoming[i]*math.sin(t) for i in (0,1)])
    points.append(b)
    return points
