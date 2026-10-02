"""Sole production owner of /cmd_vel."""
import math
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import OccupancyGrid,Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rclpy.node import Node
from lunabot_msgs.msg import SafetyStatus
from .safety_supervisor import SafetySupervisor
class SafetySupervisorNode(Node):
 def __init__(self):
  super().__init__('safety_supervisor');self.core=SafetySupervisor();self.nav=self.teleop=None;self.localization_stamp=self.sensor_stamp=self.path_stamp=None;self.slope=0.;self.obstacle=False
  self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',10);self.status_pub=self.create_publisher(SafetyStatus,'/lunabot/control/safety_status',10)
  self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.command('nav',m),10);self.create_subscription(Twist,'/cmd_vel_teleop',lambda m:self.command('teleop',m),10)
  self.create_subscription(PoseStamped,'/lunabot/localization/pose',lambda m:self.mark('localization_stamp'),10);self.create_subscription(OccupancyGrid,'/lunabot/traversability/map',lambda m:self.mark('sensor_stamp'),1);self.create_subscription(Path,'/lunabot/planning/path',lambda m:self.mark('path_stamp'),1);self.create_subscription(Imu,'/lunabot/imu',self.imu_cb,10);self.create_subscription(Bool,'/lunabot/obstacle_stop',lambda m:setattr(self,'obstacle',m.data),10)
  self.create_service(SetBool,'/lunabot/control/emergency_stop',self.estop_cb);self.create_timer(.05,self.tick)
 def now(self):return self.get_clock().now().nanoseconds/1e9
 def mark(self,name):setattr(self,name,self.now())
 def command(self,source,msg):setattr(self,source,(self.now(),msg.linear.x,msg.angular.z))
 def imu_cb(self,msg):
  q=msg.orientation;sinr=2*(q.w*q.x+q.y*q.z);cosr=1-2*(q.x*q.x+q.y*q.y);roll=math.atan2(sinr,cosr);sinp=max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)));pitch=math.asin(sinp);self.slope=math.degrees(max(abs(roll),abs(pitch)))
 def estop_cb(self,request,response):self.core.set_estop(request.data);response.success=True;response.message='emergency stop latched' if request.data else 'emergency stop cleared';return response
 def tick(self):
  decision=self.core.decide(self.now(),self.nav,self.teleop,self.localization_stamp,self.sensor_stamp,self.path_stamp,self.slope,self.obstacle);cmd=Twist();cmd.linear.x=decision.linear;cmd.angular.z=decision.angular;self.cmd_pub.publish(cmd)
  reasons={'none':SafetyStatus.REASON_NONE,'emergency_stop':SafetyStatus.REASON_EMERGENCY_STOP,'sensor_timeout':SafetyStatus.REASON_SENSOR_TIMEOUT,'localization_lost':SafetyStatus.REASON_LOCALIZATION_LOST,'path_expired':SafetyStatus.REASON_PATH_EXPIRED,'obstacle':SafetyStatus.REASON_OBSTACLE,'excessive_slope':SafetyStatus.REASON_EXCESSIVE_SLOPE,'invalid_command':SafetyStatus.REASON_INVALID_COMMAND,'command_timeout':SafetyStatus.REASON_INVALID_COMMAND};states={'safe':SafetyStatus.STATE_SAFE,'stopped':SafetyStatus.STATE_STOPPED,'estop':SafetyStatus.STATE_ESTOP,'fault':SafetyStatus.STATE_FAULT};status=SafetyStatus();status.header.stamp=self.get_clock().now().to_msg();status.state=states[decision.state];status.reason=reasons[decision.reason];status.safe_to_move=decision.safe;status.emergency_stop_latched=self.core.estop;status.command_source=decision.source;status.detail=decision.reason;self.status_pub.publish(status)
def main(args=None):
 rclpy.init(args=args);node=SafetySupervisorNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
