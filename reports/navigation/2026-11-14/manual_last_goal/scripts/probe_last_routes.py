import sys,math,json
sys.path.insert(0,'/home/yang/yang_nav/tools/nav_regress')
from plan_tunnels import pose
import rclpy
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from nav2_msgs.action import ComputePathToPose,SmoothPath

rclpy.init();node=rclpy.create_node('probe_last_routes')
pc=ActionClient(node,ComputePathToPose,'compute_path_to_pose');sc=ActionClient(node,SmoothPath,'smooth_path')
pc.wait_for_server();sc.wait_for_server()
param=node.create_client(SetParameters,'/planner_server/set_parameters');param.wait_for_service()
def wait(f,sec=20):
    rclpy.spin_until_future_complete(node,f,timeout_sec=sec)
    if not f.done():raise RuntimeError('action/service timeout')
    return f.result()
def action(c,g):
    h=wait(c.send_goal_async(g));return wait(h.get_result_async()).result
all_results=[]
for factor in [3.,2.,1.,.5,0.]:
    req=SetParameters.Request();req.parameters=[Parameter('GridBased.cost_travel_multiplier',value=factor).to_parameter_msg()]
    pr=wait(param.call_async(req))
    for x,y in [(7.6,-2.16),(12.17,2.57)]:
        g=ComputePathToPose.Goal();g.start=pose(node,x,y,-.45);g.goal=pose(node,16.9,.34,0.);g.use_start=True;g.planner_id='GridBased'
        r=action(pc,g);p=r.path
        length=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(p.poses,p.poses[1:]))
        sm=SmoothPath.Goal();sm.path=p;sm.smoother_id='zone_shaper';sm.max_smoothing_duration.sec=5;sm.check_for_collisions=False
        shaped=action(sc,sm) if p.poses else None
        row=dict(factor=factor,start=[x,y],param=[q.successful for q in pr.results],plan_error=r.error_code,length=length,min_y=min([q.pose.position.y for q in p.poses],default=0),max_y=max([q.pose.position.y for q in p.poses],default=0),shape_error=shaped.error_code if shaped else -1,poses=[[q.pose.position.x,q.pose.position.y] for q in p.poses])
        all_results.append(row);print(json.dumps({k:v for k,v in row.items() if k!='poses'}),flush=True)
open('/tmp/probe_last_routes.json','w').write(json.dumps(all_results,indent=2))
node.destroy_node();rclpy.shutdown()
