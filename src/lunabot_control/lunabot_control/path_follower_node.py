"""Production regulated pure-pursuit follower; publishes navigation requests only."""
import json,math
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import OccupancyGrid,Path
from rclpy.node import Node
from std_msgs.msg import String
from .regulated_pure_pursuit import RegulatedPurePursuit
class PathFollowerNode(Node):
 def __init__(self):
  super().__init__('regulated_path_follower')
  for name,value in [('lookahead',.8),('max_linear',.3),('max_angular',.8),('max_accel',.25),('max_decel',.5),('path_timeout',1.),('pose_timeout',.5)]:self.declare_parameter(name,value)
  p=lambda n:float(self.get_parameter(n).value);self.controller=RegulatedPurePursuit(p('lookahead'),p('max_linear'),p('max_angular'),p('max_accel'),p('max_decel'));self.path=[];self.pose=None;self.grid=None;self.path_time=None;self.pose_time=None;self.last_tick=self.get_clock().now()
  self.cmd_pub=self.create_publisher(Twist,'/cmd_vel_nav',10);self.metrics_pub=self.create_publisher(String,'/lunabot/control/tracking_metrics',10)
  self.create_subscription(Path,'/lunabot/planning/path',self.path_cb,10);self.create_subscription(PoseStamped,'/lunabot/localization/pose',self.pose_cb,10);self.create_subscription(OccupancyGrid,'/lunabot/traversability/map',self.map_cb,1);self.create_timer(.05,self.tick)
 def path_cb(self,msg):self.path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses];self.path_time=self.get_clock().now();self.controller.reset()
 def pose_cb(self,msg):
  q=msg.pose.orientation;self.pose=(msg.pose.position.x,msg.pose.position.y,math.atan2(2*q.w*q.z,1-2*(q.y*q.y+q.z*q.z)));self.pose_time=self.get_clock().now()
 def map_cb(self,msg):self.grid=msg
 def local_cost(self):
  if not self.grid or not self.pose:return 0.
  x,y,_=self.pose;c=int((x-self.grid.info.origin.position.x)/self.grid.info.resolution);r=int((y-self.grid.info.origin.position.y)/self.grid.info.resolution)
  if not(0<=r<self.grid.info.height and 0<=c<self.grid.info.width):return 1.
  value=self.grid.data[r*self.grid.info.width+c];return 1. if value<0 else min(1.,value/100.)
 def stop(self,reason):
  self.cmd_pub.publish(Twist());m=String();m.data=json.dumps({'state':'stopped','reason':reason});self.metrics_pub.publish(m)
 def tick(self):
  now=self.get_clock().now();dt=(now-self.last_tick).nanoseconds/1e9;self.last_tick=now
  if not self.path:self.stop('no_path');return
  if self.path_time is None or (now-self.path_time).nanoseconds/1e9>float(self.get_parameter('path_timeout').value):self.stop('path_timeout');return
  if self.pose_time is None or (now-self.pose_time).nanoseconds/1e9>float(self.get_parameter('pose_timeout').value):self.stop('localization_timeout');return
  terrain=self.local_cost();result=self.controller.command(self.path,self.pose,dt,roughness=terrain*.5,uncertainty=terrain*.5)
  cmd=Twist();cmd.linear.x=result.linear;cmd.angular.z=result.angular;self.cmd_pub.publish(cmd);m=String();m.data=json.dumps({'state':'tracking','cross_track_error':result.cross_track_error,'heading_error':result.heading_error,'curvature':result.curvature,'target_index':result.target_index});self.metrics_pub.publish(m)
def main(args=None):
 rclpy.init(args=args);node=PathFollowerNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
