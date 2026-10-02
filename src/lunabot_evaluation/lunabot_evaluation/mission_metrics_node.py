"""ROS mission-metrics recorder; contact input must originate from physics."""
import json,os,tempfile
import rclpy
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from rclpy.node import Node
from lunabot_msgs.msg import MissionMetrics,ReplanEvent
from .mission_metrics import MissionMetricsCollector
class MissionMetricsNode(Node):
 def __init__(self):
  super().__init__('mission_metrics');self.declare_parameter('trial_output','evidence/trials/trial.json');self.c=MissionMetricsCollector();self.c.start(self.now());self.last=None
  self.pub=self.create_publisher(MissionMetrics,'/lunabot/evaluation/mission_metrics',10)
  self.create_subscription(Odometry,'/lunabot/odom',self.odom,10);self.create_subscription(Path,'/lunabot/planning/path',self.path,10);self.create_subscription(ReplanEvent,'/lunabot/planning/replan_event',lambda m:self.c.replan(m.replan_time_ms),10);self.create_subscription(String,'/lunabot/control/tracking_metrics',self.tracking,10);self.create_subscription(String,'/lunabot/simulation/contact',self.contact,10);self.create_subscription(String,'/lunabot/mission/result',self.finish,10);self.create_timer(1.,self.publish)
 def now(self):return self.get_clock().now().nanoseconds/1e9
 def odom(self,m):self.c.pose(self.now(),m.pose.pose.position.x,m.pose.pose.position.y)
 def path(self,m):self.c.set_plan([(p.pose.position.x,p.pose.position.y) for p in m.poses])
 def tracking(self,m):
  try:self.c.tracking_error(float(json.loads(m.data)['cross_track_error']))
  except (ValueError,KeyError,TypeError,json.JSONDecodeError):pass
 def contact(self,m):
  try:
   d=json.loads(m.data);self.c.contact(float(d.get('stamp',self.now())),str(d['object']),bool(d['active']),float(d.get('relative_speed',0)),float(d.get('impulse',0)))
  except (ValueError,KeyError,TypeError,json.JSONDecodeError):self.get_logger().warning('invalid contact event')
 def finish(self,m):
  success=m.data.strip().upper()=='SUCCESS';self.c.finish(self.now(),success,'' if success else m.data);self.publish();self.write()
 def message(self):
  d=self.c.result();m=MissionMetrics();m.header.stamp=self.get_clock().now().to_msg()
  for key in ('complete','success','failure_reason','planned_path_length_m','executed_path_length_m','mission_duration_s','path_efficiency','accumulated_traversability_cost','minimum_clearance_m','replan_count','mean_replan_time_ms','max_replan_time_ms','collision_count','collision_duration_s','controller_cross_track_rmse_m','terrain_distance_m'):setattr(m,key,d[key])
  return m
 def publish(self):self.pub.publish(self.message())
 def write(self):
  path=str(self.get_parameter('trial_output').value);os.makedirs(os.path.dirname(path) or '.',exist_ok=True);fd,tmp=tempfile.mkstemp(dir=os.path.dirname(path) or '.',prefix='.trial-',text=True)
  with os.fdopen(fd,'w') as f:json.dump(self.c.result(),f,indent=2,sort_keys=True);f.write('\n')
  os.replace(tmp,path)
def main(args=None):
 rclpy.init(args=args);node=MissionMetricsNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
