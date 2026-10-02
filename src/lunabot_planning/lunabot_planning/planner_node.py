"""ROS 2 incremental D*-Lite planner consuming only the production map."""
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy
from lunabot_msgs.msg import PlannerStatus,ReplanEvent
from .dstar_lite import DStarLite
class PlannerNode(Node):
 def __init__(self):
  super().__init__('dstar_lite_planner');self.grid=None;self.planner=None;self.start=None;self.goal=None;self.revision=0;self.path_cells=set()
  qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.path_pub=self.create_publisher(Path,'/lunabot/planning/path',qos);self.status_pub=self.create_publisher(PlannerStatus,'/lunabot/planning/status',qos);self.event_pub=self.create_publisher(ReplanEvent,'/lunabot/planning/replan_event',qos)
  self.create_subscription(OccupancyGrid,'/lunabot/traversability/map',self.map_callback,1);self.create_subscription(PoseStamped,'/lunabot/localization/pose',self.start_callback,10);self.create_subscription(PoseStamped,'/lunabot/planning/goal',self.goal_callback,10)
 def cell(self,pose):
  p=pose.pose.position;return (int((p.y-self.grid.info.origin.position.y)/self.grid.info.resolution),int((p.x-self.grid.info.origin.position.x)/self.grid.info.resolution))
 def map_callback(self,msg):
  if self.grid and (msg.info.width,msg.info.height)!=(self.grid.info.width,self.grid.info.height):self.planner=None
  old=list(self.grid.data) if self.grid else None;self.grid=msg
  if self.planner and old is not None:
   changes={divmod(i,msg.info.width):value for i,value in enumerate(msg.data) if value!=old[i]}
   newly_lethal={cell for cell,value in changes.items() if value>=100 and old[cell[0]*msg.info.width+cell[1]]<100}
   invalidated=bool(newly_lethal & self.path_cells)
   self.planner.update_costs(changes);self.revision+=1
   reason=ReplanEvent.REASON_OBSTACLE if invalidated else ReplanEvent.REASON_TERRAIN_COST
   self.plan(len(changes),reason,invalidated)
  elif self.start is not None and self.goal is not None:self.initialize()
 def start_callback(self,msg):
  if not self.grid:return
  cell=self.cell(msg)
  if self.planner:self.planner.move_start(cell);self.start=cell;self.plan(0,ReplanEvent.REASON_START_MOVED)
  else:self.start=cell
 def goal_callback(self,msg):
  if not self.grid:return
  self.goal=self.cell(msg);self.initialize()
 def initialize(self):
  if self.grid and self.start is not None and self.goal is not None:self.planner=DStarLite(self.grid.info.width,self.grid.info.height,self.grid.data,self.start,self.goal);self.plan(0,ReplanEvent.REASON_GOAL_CHANGED)
 def plan(self,changed,reason,invalidated=False):
  if not self.planner:return
  begin=time.perf_counter();success=self.planner.compute_shortest_path();cells=self.planner.path() if success else [];elapsed=(time.perf_counter()-begin)*1000
  path=Path();path.header=self.grid.header
  for row,col in cells:
   pose=PoseStamped();pose.header=path.header;pose.pose.position.x=self.grid.info.origin.position.x+(col+.5)*self.grid.info.resolution;pose.pose.position.y=self.grid.info.origin.position.y+(row+.5)*self.grid.info.resolution;pose.pose.orientation.w=1.;path.poses.append(pose)
  self.path_cells=set(cells);self.path_pub.publish(path);status=PlannerStatus();status.header=path.header;status.state=PlannerStatus.STATE_PATH_READY if success else PlannerStatus.STATE_NO_PATH;status.planner_id='dstar_lite';status.success=success;status.incremental=True;status.planning_time_ms=elapsed;status.expanded_nodes=self.planner.expanded;status.path_cells=len(cells);status.path_length_m=max(0,len(cells)-1)*self.grid.info.resolution;status.accumulated_cost=self.planner.path_cost(cells) if cells else 0.;status.detail='incremental repair' if changed else 'plan';self.status_pub.publish(status)
  event=ReplanEvent();event.header=path.header;event.reason=reason;event.success=success;event.active_path_invalidated=invalidated;event.map_update_stamp=path.header.stamp;event.replan_start_stamp=path.header.stamp;event.replan_finish_stamp=self.get_clock().now().to_msg();event.changed_cells=changed;event.revision=self.revision;event.replan_time_ms=elapsed;event.total_latency_ms=elapsed;event.detail=status.detail;self.event_pub.publish(event)
def main(args=None):
 rclpy.init(args=args);node=PlannerNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
