"""Check every published service route against this PIE world's pavement."""
import math
from types import SimpleNamespace

from .road_guard import body_polygon, swept_body


def verify_service_surfaces(geometry,surface,rear_z_m=1.29,progress=None):
    network=geometry.network;seen=set();sweeps=set();routes=0
    for origin in network.services:
        for target,row in network.services.items():
            if origin==target or row['kind']=='staging':continue
            route=network.route(origin,target)
            if not route:raise RuntimeError('Unreachable service route '+origin+' -> '+target)
            plan=geometry.plan(SimpleNamespace(origin=origin,destination=target,route=route));previous=None
            for i in range(math.ceil(plan.length/.5)+1):
                station=min(plan.length,i*.5);x,y=plan.point(station);h=plan.heading(station)
                pose=(x,y,h);key=tuple(round(v,5) for v in pose)
                if key not in seen:
                    seen.add(key)
                    if not surface.covers(body_polygon(*pose),rear_z_m):
                        raise RuntimeError('Service route body leaves native pavement: '+repr((origin,target,station,pose)))
                if previous is not None:
                    sweep_key=(tuple(round(v,5) for v in previous),key)
                    if sweep_key not in sweeps:
                        sweeps.add(sweep_key)
                        if not surface.covers(swept_body(previous,pose),rear_z_m):
                            raise RuntimeError('Service route body sweep leaves native pavement: '+repr((origin,target,station,pose)))
                previous=pose
            routes+=1;geometry.cache.clear()
        if progress:progress(routes,len(seen),len(sweeps))
    return dict(status='PASS',routes=routes,unique_body_poses=len(seen),unique_body_sweeps=len(sweeps),
                maximum_step_m=.5,scope='Nominal published service routes, body polygons and conservative consecutive-pose hulls; not a proof of all native reachable states.')
