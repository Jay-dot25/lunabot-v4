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
  super().__init__('replan_verifier');self.core=ReplanCorrelation();self.geometry=None;self.initialized=False;self.pending_path=None
  self.pub=self.create_publisher(String,'/lunabot/planning/replan_verification',10)
  self.create_subscription(Path,'/lunabot/planning/path',self.path_cb,10);self.create_subscription(OccupancyGrid,'/lunabot/obstacles/map',self.detect_cb,10);self.create_subscription(OccupancyGrid,'/lunabot/traversability/map',self.map_cb,10);self.create_subscription(ReplanEvent,'/lunabot/planning/replan_event',self.event_cb,10)
 def stamp(self,msg):return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
 def cells(self,path):
  if not self.geometry:return []
  return [(int((p.pose.position.y-self.geometry.info.origin.position.y)/self.geometry.info.resolution),int((p.pose.position.x-self.geometry.info.origin.position.x)/self.geometry.info.resolution)) for p in path.poses]
 def path_cb(self,msg):
  cells=self.cells(msg)
  if not self.initialized:self.core.set_initial_path(cells);self.initialized=True
  else:self.pending_path=(self.stamp(msg),cells)
 def occupied_cells(self,msg):
  if not self.geometry:return []
  result=[]
  for i,value in enumerate(msg.data):
   if value<100:continue
   row,col=divmod(i,msg.info.width);x=msg.info.origin.position.x+(col+.5)*msg.info.resolution;y=msg.info.origin.position.y+(row+.5)*msg.info.resolution
   rr=int((y-self.geometry.info.origin.position.y)/self.geometry.info.resolution);cc=int((x-self.geometry.info.origin.position.x)/self.geometry.info.resolution)
   if 0<=rr<self.geometry.info.height and 0<=cc<self.geometry.info.width:result.append((rr,cc))
  return result
 def detect_cb(self,msg):self.core.sensor_detection(self.stamp(msg),self.occupied_cells(msg))
 def map_cb(self,msg):
  self.geometry=msg;cells=[divmod(i,msg.info.width) for i,value in enumerate(msg.data) if value>=100];self.core.lethal_update(self.stamp(msg),cells)
 def event_cb(self,msg):
  reason='obstacle' if msg.reason==ReplanEvent.REASON_OBSTACLE and msg.active_path_invalidated else 'other'
  if self.core.replan_requested(self.stamp(msg),reason) and self.pending_path:
   stamp,cells=self.pending_path;self.core.path_published(max(stamp,self.stamp(msg)),cells);self.pending_path=None;self.publish()
 def publish(self):msg=String();msg.data=json.dumps(self.core.result(),sort_keys=True);self.pub.publish(msg)
def main(args=None):
 rclpy.init(args=args);node=ReplanVerifier()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
