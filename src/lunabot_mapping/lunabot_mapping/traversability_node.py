"""Create production traversability and explanation maps from semantic layers."""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer,Subscriber
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
from .traversability import DEFAULT,build_traversability
class TraversabilityNode(Node):
 def __init__(self):
  super().__init__('traversability_node');self.bridge=CvBridge()
  for key,value in DEFAULT.items():self.declare_parameter(key,value)
  qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.cost_pub=self.create_publisher(OccupancyGrid,'/lunabot/traversability/map',qos);self.reason_pub=self.create_publisher(OccupancyGrid,'/lunabot/traversability/reason',qos)
  subs=[Subscriber(self,OccupancyGrid,'/lunabot/semantic_map/class'),Subscriber(self,OccupancyGrid,'/lunabot/semantic_map/confidence'),Subscriber(self,Image,'/lunabot/semantic_map/elevation'),Subscriber(self,Image,'/lunabot/semantic_map/stale')]
  self.sync=ApproximateTimeSynchronizer(subs,5,.05);self.sync.registerCallback(self.callback)
 def callback(self,classes,confidence,elevation,stale):
  width,height=classes.info.width,classes.info.height
  if confidence.info.width!=width or confidence.info.height!=height:return
  z=self.bridge.imgmsg_to_cv2(elevation,'32FC1').reshape(-1).tolist();stale_values=(self.bridge.imgmsg_to_cv2(stale,'32FC1').reshape(-1)>0.5).tolist();cfg={key:self.get_parameter(key).value for key in DEFAULT}
  result=build_traversability([0 if value<0 else value for value in classes.data],[value/100 for value in confidence.data],z,stale_values,width,height,classes.info.resolution,cfg)
  for publisher,data in ((self.cost_pub,result['cost']),(self.reason_pub,result['reason'])):
   msg=OccupancyGrid();msg.header=classes.header;msg.info=classes.info;msg.data=[int(value) for value in data];publisher.publish(msg)
def main(args=None):
 rclpy.init(args=args);node=TraversabilityNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
