"""Production obstacle-event correlator; motion-only path changes never pass."""
import json
import rclpy
from nav_msgs.msg import OccupancyGrid,Path
from rclpy.node import Node
from std_msgs.msg import String
from lunabot_msgs.msg import ReplanEvent
from .replan_correlation import ReplanCorrelation
class ReplanVerifier(Node):
 def __init__(self):
  super().__init__('replan_verifier');self.core=ReplanCorrelation();self.geometry=None;self.initialized=False
  self.pub=self.create_publisher(String,'/lunabot/planning/replan_verification',10)
  self.create_subscription(Path,'/lunabot/planning/path',self.path_cb,10);self.create_subscription(OccupancyGrid,'/lunabot/obstacles/map',self.detect_cb,10);self.create_subscription(OccupancyGrid,'/lunabot/traversability/map',self.map_cb,10);self.create_subscription(ReplanEvent,'/lunabot/planning/replan_event',self.event_cb,10)
 def stamp(self,msg):return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
 def cells(self,path):
  if not self.geometry:return []
  return [(int((p.pose.position.y-self.geometry.info.origin.position.y)/self.geometry.info.resolution),int((p.pose.position.x-self.geometry.info.origin.position.x)/self.geometry.info.resolution)) for p in path.poses]
 def path_cb(self,msg):
  cells=self.cells(msg)
  if not self.initialized:self.core.set_initial_path(cells);self.initialized=True
  else:self.core.path_published(self.stamp(msg),cells);self.publish()
 def detect_cb(self,msg):
  self.geometry=msg;cells=[divmod(i,msg.info.width) for i,value in enumerate(msg.data) if value>=100];self.core.sensor_detection(self.stamp(msg),cells)
 def map_cb(self,msg):
  self.geometry=msg;cells=[divmod(i,msg.info.width) for i,value in enumerate(msg.data) if value>=100];self.core.lethal_update(self.stamp(msg),cells)
 def event_cb(self,msg):
  reason='obstacle' if msg.reason==ReplanEvent.REASON_OBSTACLE else 'other';self.core.replan_requested(self.stamp(msg),reason)
 def publish(self):msg=String();msg.data=json.dumps(self.core.result(),sort_keys=True);self.pub.publish(msg)
def main(args=None):
 rclpy.init(args=args);node=ReplanVerifier()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
