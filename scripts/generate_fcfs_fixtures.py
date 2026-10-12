#!/usr/bin/env python3
"""Generate temporary algorithm fixtures; these are not final park layouts."""
import json
from math import atan2, cos, hypot, pi, sin
from pathlib import Path


def generate(queue=False,blocked=False):
    nodes={}
    edges=[]
    def node(key,point,kind='WAYPOINT'):
        nodes[key]={'node_id':key,'node_type':kind,'position':list(point)}
    def edge(key,a,b,points=None,capacity=3,speed=.25):
        line=points or [nodes[a]['position'],nodes[b]['position']]
        length=sum(hypot(q[0]-p[0],q[1]-p[1]) for p,q in zip(line,line[1:]))
        capacity=min(capacity,int(length/(.12+.05)))
        edges.append({'edge_id':key,'from_node':a,'to_node':b,'centerline':line,
                      'width':.3,'speed_limit':speed,'storage_capacity':capacity})
    def curve(a,b,entry_heading,exit_heading):
        p0,p3=nodes[a]['position'],nodes[b]['position']
        reach=min(.22,((p3[0]-p0[0])**2+(p3[1]-p0[1])**2)**.5*.40)
        p1=[p0[0]+reach*cos(entry_heading),p0[1]+reach*sin(entry_heading)]
        p2=[p3[0]-reach*cos(exit_heading),p3[1]-reach*sin(exit_heading)]
        values=[]
        for i in range(41):
            t=i/40
            values.append([round((1-t)**3*p0[j]+3*(1-t)**2*t*p1[j]+3*(1-t)*t*t*p2[j]+t**3*p3[j],8) for j in range(2)])
        values[0],values[-1]=p0,p3
        return values
    incoming={'W':(.65,.90),'N':(.90,1.35),'E':(1.35,1.10),'S':(1.10,.65)}
    outgoing={'W':(.65,1.10),'N':(1.10,1.35),'E':(1.35,.90),'S':(.90,.65)}
    sources={'W':(0,.90),'N':(.90,2),'E':(2,1.10),'S':(1.10,0)}
    destinations={'W':(-.2,1.10),'N':(1.10,2.2),'S':(.90,-.2)}
    in_heading={'W':0,'N':-pi/2,'E':pi,'S':pi/2}
    out_heading={'W':pi,'N':pi/2,'E':0,'S':-pi/2}
    for arm in incoming:
        node('in_'+arm,incoming[arm],'INTERSECTION')
        node('out_'+arm,outgoing[arm],'INTERSECTION')
        node('src_'+arm,sources[arm],'GATE')
    if queue:
        node('rear',(-.10,.90),'GATE')
        node('middle',(.15,.90),'GATE')
        node('front',(.40,.90),'GATE')
        edge('approach_W','front','in_W',capacity=2)
        edge('rear_middle','rear','middle',capacity=2)
        edge('middle_front','middle','front',capacity=2)
    else:
        edge('approach_W','src_W','in_W')
    for arm in ('N','E','S'):
        edge('approach_'+arm,'src_'+arm,'in_'+arm)
    for arm,point in destinations.items():
        node('dest_'+arm,point,'WAREHOUSE')
        edge('exit_'+arm,'out_'+arm,'dest_'+arm)
    node('fork_in',(2.15,.90),'INTERSECTION')
    if blocked:
        node('blocker_start',(1.75,.90),'GATE')
        edge('exit_E','out_E','blocker_start',capacity=1)
        edge('blocker_fork','blocker_start','fork_in',capacity=2)
    else:
        edge('exit_E','out_E','fork_in',capacity=3)
    cross={}
    for arm in incoming:
        for destination in outgoing:
            if arm==destination:
                continue
            key=f'cross_{arm}_{destination}'
            edge(key,'in_'+arm,'out_'+destination,curve('in_'+arm,'out_'+destination,in_heading[arm],out_heading[destination]),1,.18)
            cross[key]='exit_'+destination
    fork={}
    for i,y in enumerate((.60,.90,1.20)):
        port=f'fork_out_{i}'
        bay=f'bay_E_{i}'
        node(port,(2.55,y),'INTERSECTION')
        node(bay,(3,y),'WAREHOUSE')
        connector=f'fork_{i}'
        parking=f'parking_E_{i}'
        edge(connector,'fork_in',port,curve('fork_in',port,0,0),1,.18)
        edge(parking,port,bay,capacity=1,speed=.2)
        fork[connector]=parking
    movements=[]
    for a in edges:
        for b in edges:
            if a['to_node']!=b['from_node'] or a['from_node']==b['to_node']:
                continue
            p0,p1,p2=a['centerline'][-2],a['centerline'][-1],b['centerline'][1]
            angle=(atan2(p2[1]-p1[1],p2[0]-p1[0])-atan2(p1[1]-p0[1],p1[0]-p0[0])+pi)%(2*pi)-pi
            movements.append({'movement_id':a['edge_id']+'__'+b['edge_id'],'incoming_edge':a['edge_id'],
                              'outgoing_edge':b['edge_id'],'maneuver':'STRAIGHT' if abs(angle)<.2 else 'LEFT' if angle>0 else 'RIGHT',
                              'reference_path':[p0,p1,p2],'conflict_zones':[],'speed_limit':min(a['speed_limit'],b['speed_limit'])})
    graph={'description':'TEMPORARY Phase 4 FCFS fixture, not final park environment',
           'nodes':list(nodes.values()),'edges':edges,'movements':movements}
    controls=[{'intersection_id':'cross','bounds':[.65,.65,1.35,1.35],'connector_exits':cross},
              {'intersection_id':'fork','bounds':[2.15,.45,2.55,1.35],'connector_exits':fork}]
    stations=[{'station_id':key,'berth_nodes':[key],'service_capacity':1,'waiting_capacity':0,'service_time':1}
              for key in list('dest_'+arm for arm in destinations)+[f'bay_E_{i}' for i in range(3)]]
    return graph,controls,stations


def main():
    root=Path(__file__).resolve().parents[1]
    cases=[]
    for name,queue,blocked,turns in [('four_way',False,False,False),('turns',False,False,True),
                                     ('same_lane_fifo',True,False,False),('downstream_full',False,True,False)]:
        graph,controls,stations=generate(queue,blocked)
        homes={}
        tasks=[]
        if queue:
            assignments=[('car_00','rear','bay_E_2'),('car_01','middle','bay_E_1'),('car_02','front','bay_E_0')]
        elif turns:
            assignments=[('car_00','src_W','dest_S'),('car_01','src_N','bay_E_0'),
                         ('car_02','src_E','dest_N'),('car_03','src_S','dest_W')]
        else:
            assignments=[('car_00','src_W','bay_E_0'),('car_01','src_N','dest_S'),
                         ('car_02','src_E','dest_W'),('car_03','src_S','dest_N')]
        if blocked:
            assignments.append(('blocker','blocker_start','bay_E_2'))
        for vehicle,origin,destination in assignments:
            homes[vehicle]=origin
            tasks.append({'task_id':name+'_'+vehicle,'vehicle_id':vehicle,'origin':origin,'destination':destination,
                          'release_time':8 if vehicle=='blocker' else 0})
        directory=root/'config/traffic'
        directory.mkdir(parents=True,exist_ok=True)
        topology=root/'config/topology'/('fcfs_'+name+'.json')
        topology.write_text(json.dumps(graph,indent=2)+'\n')
        scenario={'description':'Algorithm verification only; final environment supplied later by user',
                  'intersections':controls,'stations':stations,'vehicle_nodes':homes,'tasks':tasks,
                  'initial_edges':{'blocker':'exit_E'} if blocked else {},
                  'checks':{'parallel_motion':True,'same_lane_fifo':queue,'downstream_full':blocked,
                            'independent_junctions':queue}}
        scenario_path=directory/(name+'.json')
        scenario_path.write_text(json.dumps(scenario,indent=2)+'\n')
        cases.append({'name':name,'map':'../topology/'+topology.name, 'scenario':'../traffic/'+scenario_path.name})
    config={'simulation':{'dt':.02,'duration':120,'control_hz':20,'task_hz':5,'logging_hz':20,'max_lateral_error':.06,
                           'arrival_tolerance':.012,'stopped_speed':.005,'safety_margin':.015,'algorithm':'dijkstra',
                           'vehicle':{'v_max':.5,'omega_max':3,'a_max':.5,'deceleration':.7,'length':.12,'width':.08,'wheel_track':.07},
                           'tracking':{'cruise_speed':.25,'lookahead_base':.08,'lookahead_gain':.3,'lateral_gain':.5,'heading_gain':.3,'lateral_acceleration':.3}},
            'rules':{'request_distance':.35,'stop_offset':.12,'entry_gap':.2,'standstill_gap':.05,'time_headway':.4},'cases':cases}
    (root/'config/simulation/fcfs.json').write_text(json.dumps(config,indent=2)+'\n')
    print('Generated four temporary FCFS fixtures.')


if __name__=='__main__':
    main()
