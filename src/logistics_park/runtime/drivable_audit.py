"""Audit every recorded body and inter-frame sweep, including service access."""
from .road_guard import body_polygon, swept_body


def audit_frames(frames,surface):
    previous={};expected=None;samples=sweeps=0;flags=[];maximum=0.;frames_seen=0
    for frame in frames:
        positions=frame['positions'];heights=frame['rear_z_m'];frames_seen+=1
        if expected is None:expected=set(positions)
        if not expected or set(positions)!=expected or set(heights)!=expected:
            raise RuntimeError('Incomplete native road audit frame')
        for name,values in positions.items():
            x,y,h=values[:3];pose=(x,y,h);samples+=1
            uncovered=surface.uncovered_area(body_polygon(*pose),heights[name])
            maximum=max(maximum,uncovered)
            if uncovered>.002:flags.append(dict(frame=frames_seen,vehicle=name,kind='body',uncovered_m2=uncovered))
            if name in previous:
                sweeps+=1
                uncovered=surface.uncovered_area(swept_body(previous[name],pose),heights[name])
                maximum=max(maximum,uncovered)
                if uncovered>.002:flags.append(dict(frame=frames_seen,vehicle=name,kind='sweep',uncovered_m2=uncovered))
            previous[name]=pose
    if not frames_seen:raise RuntimeError('Empty native road audit')
    return dict(status='FAIL' if flags else 'PASS',frames=frames_seen,body_samples=samples,body_sweeps=sweeps,
                maximum_uncovered_area_m2=maximum,flag_count=len(flags),flags=flags[:100],
                scope='All retained fleet bodies and consecutive native-frame swept hulls on the loaded approved pavement triangles; no road-kind or attachment exclusions.')
