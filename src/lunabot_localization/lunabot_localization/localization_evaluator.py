"""Synchronize filtered and evaluation-only ground-truth pose and report metrics."""
from __future__ import annotations
import json
import rclpy
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer,Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from .metrics import evaluate_trajectory

def yaw(q):
 import math
 return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class LocalizationEvaluator(Node):
 def __init__(self):
  super().__init__('localization_evaluator');self.declare_parameter('estimate_topic','/odometry/filtered');self.declare_parameter('ground_truth_topic','/lunabot/ground_truth/pose');self.declare_parameter('report_path','');self.estimated=[];self.truth=[]
  estimate=Subscriber(self,Odometry,str(self.get_parameter('estimate_topic').value));truth=Subscriber(self,PoseStamped,str(self.get_parameter('ground_truth_topic').value));self.sync=ApproximateTimeSynchronizer([estimate,truth],50,.05);self.sync.registerCallback(self.callback)
 def callback(self,estimate,truth):
  stamp=estimate.header.stamp.sec*1000000000+estimate.header.stamp.nanosec;e=estimate.pose.pose;t=truth.pose
  self.estimated.append((stamp,e.position.x,e.position.y,yaw(e.orientation)));self.truth.append((stamp,t.position.x,t.position.y,yaw(t.orientation)))
 def report(self):
  if len(self.truth)<2:return None
  result=evaluate_trajectory(self.estimated,self.truth);text=json.dumps(result,indent=2,sort_keys=True);self.get_logger().info(text);path=str(self.get_parameter('report_path').value)
  if path:open(path,'w',encoding='utf-8').write(text+'\n')
  return result
def main(args=None):
 rclpy.init(args=args);node=LocalizationEvaluator()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.report();node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
