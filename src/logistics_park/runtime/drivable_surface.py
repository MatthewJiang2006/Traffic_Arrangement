"""Full polygon coverage by the actual loaded pavement triangles (stdlib only).

Checking corners alone misses holes and gaps. Subtract convex triangles from
the body polygon until no uncovered area remains. Overlapping road, junction,
hardstand and approved ramp meshes form one surface without lane-only seams.
"""
import math


def area(polygon):
    return abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(polygon,polygon[1:]+polygon[:1]))) / 2


def cross(a,b,p):
    return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])


def clip(polygon,a,b,inside=True):
    if not polygon:return []
    result=[];previous=polygon[-1];old=cross(a,b,previous)*(1 if inside else -1)
    for point in polygon:
        value=cross(a,b,point)*(1 if inside else -1)
        if (value>=0)!=(old>=0):
            fraction=old/(old-value)
            result.append((previous[0]+fraction*(point[0]-previous[0]),previous[1]+fraction*(point[1]-previous[1])))
        if value>=0:result.append(point)
        previous,old=point,value
    return result


def subtract_triangle(polygon,triangle):
    if (max(p[0] for p in polygon)<min(p[0] for p in triangle) or min(p[0] for p in polygon)>max(p[0] for p in triangle)
            or max(p[1] for p in polygon)<min(p[1] for p in triangle) or min(p[1] for p in polygon)>max(p[1] for p in triangle)):
        return [polygon]
    remains=[];candidate=polygon
    for a,b in zip(triangle,triangle[1:]+triangle[:1]):
        outside=clip(candidate,a,b,False)
        if len(outside)>=3 and area(outside)>1e-10:remains.append(outside)
        candidate=clip(candidate,a,b)
        if len(candidate)<3:break
    return remains


def convex_hull(points):
    points=sorted(set(points))
    if len(points)<=2:return points
    lower=[];upper=[]
    for p in points:
        while len(lower)>1 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>1 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


class DrivableSurface:
    def __init__(self,snapshot,cell_m=10.,*,accelerated=True):
        if snapshot.get('schema')!=1:raise ValueError('Unsupported drivable surface schema')
        if not math.isfinite(cell_m) or cell_m<=0:raise ValueError('Positive surface cell size required')
        self.cell_m=cell_m;self.triangles=[];self.cells={};seen=set();self.actors=set()
        for surface in snapshot['surfaces']:
            for values in surface['triangles']:
                if len(values)!=9 or not all(math.isfinite(v) for v in values):raise ValueError('Invalid pavement triangle')
                xyz=[tuple(values[i:i+3]) for i in (0,3,6)];points=[p[:2] for p in xyz]
                if area(points)<1e-9:continue
                if cross(*points)<0:points.reverse()
                key=tuple(sorted(tuple(round(v,6) for v in p) for p in xyz))
                if key in seen:continue
                seen.add(key);self.actors.add(surface['actor'])
                x0,x1=min(p[0] for p in points),max(p[0] for p in points)
                y0,y1=min(p[1] for p in points),max(p[1] for p in points)
                z0,z1=min(p[2] for p in xyz),max(p[2] for p in xyz)
                index=len(self.triangles);self.triangles.append((points,(x0,y0,x1,y1),z0,z1))
                for x in range(math.floor(x0/cell_m),math.floor(x1/cell_m)+1):
                    for y in range(math.floor(y0/cell_m),math.floor(y1/cell_m)+1):self.cells.setdefault((x,y),[]).append(index)
        if not self.triangles:raise ValueError('No approved drivable surface triangles')
        self.accelerator=None
        if accelerated:
            from .geos_surface import GeosCoverage
            try:self.accelerator=GeosCoverage()
            except OSError:pass

    def uncovered_area(self,polygon,rear_z_m,*,tolerance_m2=.002):
        if len(polygon)<3 or not math.isfinite(rear_z_m):raise ValueError('Invalid body polygon or height')
        if not all(math.isfinite(v) for p in polygon for v in p):raise ValueError('Nonfinite body polygon')
        x0,x1=min(p[0] for p in polygon),max(p[0] for p in polygon)
        y0,y1=min(p[1] for p in polygon),max(p[1] for p in polygon)
        candidates=set()
        for x in range(math.floor(x0/self.cell_m),math.floor(x1/self.cell_m)+1):
            for y in range(math.floor(y0/self.cell_m),math.floor(y1/self.cell_m)+1):candidates.update(self.cells.get((x,y),()))
        if self.accelerator:
            eligible=sorted(i for i in candidates if self.triangles[i][3]>=rear_z_m-.8 and self.triangles[i][2]<=rear_z_m+.25)
            return self.accelerator.uncovered(polygon,eligible,self.triangles)
        remaining=[list(polygon)]
        for index in sorted(candidates):
            triangle,(a,b,c,d),z0,z1=self.triangles[index]
            if c<x0 or a>x1 or d<y0 or b>y1 or z1<rear_z_m-.8 or z0>rear_z_m+.25:continue
            remaining=[piece for polygon in remaining for piece in subtract_triangle(polygon,triangle)]
            if not remaining:return 0.
        return sum(area(p) for p in remaining)

    def covers(self,polygon,rear_z_m):
        return self.uncovered_area(polygon,rear_z_m)<=.002
