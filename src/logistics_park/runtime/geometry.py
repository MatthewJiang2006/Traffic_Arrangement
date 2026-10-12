from logistics_park.map.geometry import Polyline


def project(point, path):
    p=Polyline(tuple(tuple(p) for p in path)).project(*point[:2])
    return p.distance,p.s


def at_station(path, station):
    return Polyline(tuple(tuple(p) for p in path)).at(station)[0]


def euclidean_xy(a,b):
    from math import hypot
    return hypot(a[0]-b[0],a[1]-b[1])


def support(audit,axis):
    from math import radians,cos,sin
    ends=[]
    for body in audit['bodies']:
        angle=radians(body['yaw_deg']);x,y=body['half_size_m'][:2]
        extent=abs(cos(angle))*x+abs(sin(angle))*y if axis==0 else abs(sin(angle))*x+abs(cos(angle))*y
        centre=body['centre_ue_m'][axis];ends.extend((centre-extent,centre+extent))
    return min(ends),max(ends)
