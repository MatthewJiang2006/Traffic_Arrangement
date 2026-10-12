#!/usr/bin/env python3
"""Annotate the surveyed UE road map with the four configured terminal sites."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
config=json.loads((ROOT/'config/traffic/industrialarea_ue_test_sites.json').read_text())
survey=json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text())
candidates=json.loads((ROOT/'runs/ue_scene/test_sites/path_candidates.json').read_text())
probe=json.loads((ROOT/'runs/ue_scene/test_sites/path_probe.json').read_text())
svg=(ROOT/'docs/industrialarea_ue_roadgraph.svg').read_text()
svg=svg.replace('IndustrialArea UE road graph — both copies as one park','A–D test sites · UE +X right, +Y up')
svg=svg.replace('UE world XY (m); actual lane counts; turn paths require vehicle calibration','Camera rotation does not change world axes; circles: start / finish; green: proposed U-turn path')
parts=[]
def xy(p):return 70+(p[0]+109)*4.2,75+(312-p[1])*4.2
for name,site in config['sites'].items():
 rows=[r for r in probe[name] if r['pass']]
 chosen=min(rows,key=lambda r:(abs(candidates[name][r['candidate']]['radius_m']-3.8),candidates[name][r['candidate']].get('length_m',0)))
 path=candidates[name][chosen['candidate']]['rear_path']
 points=' '.join(f'{x:.1f},{y:.1f}' for x,y in map(xy,path))
 parts.append(f'<polyline points="{points}" fill="none" stroke="#059669" stroke-width="3" opacity="0.85"/>')
 for role,color in [('start','#0891b2'),('finish','#f59e0b')]:
  x,y=xy(site[role]['xy_m'])
  parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="{color}" stroke="white" stroke-width="2"><title>{name} {role}</title></circle>')
 x,y=xy(site['finish']['xy_m']);labelx=x+17 if name in ('A','C') else x-60
 parts.append(f'<rect x="{labelx-4:.1f}" y="{y-17:.1f}" width="48" height="29" rx="5" fill="white" stroke="#334155"/>')
 parts.append(f'<text x="{labelx+9:.1f}" y="{y+5:.1f}" font-family="sans-serif" font-size="22" font-weight="bold" fill="#0f172a">{name}</text>')
svg=svg.replace('</svg>','\n'.join(parts)+'\n</svg>')
(ROOT/'docs/ue_test_sites.svg').write_text(svg)
