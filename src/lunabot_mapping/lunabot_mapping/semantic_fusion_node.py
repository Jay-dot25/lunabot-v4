"""Synchronized calibrated semantic/depth fusion into a bounded local map."""
from __future__ import annotations
import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer,Subscriber
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo,Image
from tf2_ros import Buffer,TransformException,TransformListener
from .semantic_fusion import SemanticFusionGrid,project_pixel

def matrix_from_transform(transform):
 q=transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
 rotation=(1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))
 t=transform.translation
 return (rotation[0],rotation[1],rotation[2],t.x,rotation[3],rotation[4],rotation[5],t.y,rotation[6],rotation[7],rotation[8],t.z,0.,0.,0.,1.)
class SemanticFusionNode(Node):
 def __init__(self):
  super().__init__('semantic_fusion');defaults={'class_topic':'/lunabot/terrain/class_image','confidence_topic':'/lunabot/terrain/confidence','depth_topic':'/lunabot/depth/image_raw','camera_info_topic':'/lunabot/depth/camera_info','map_frame':'map','width':200,'height':200,'resolution':.1,'origin_x':-10.,'origin_y':-10.,'sample_stride':4,'sync_slop_s':.08}
  for key,value in defaults.items():self.declare_parameter(key,value)
  self.bridge=CvBridge();self.grid=SemanticFusionGrid(int(self.get_parameter('width').value),int(self.get_parameter('height').value),float(self.get_parameter('resolution').value),(float(self.get_parameter('origin_x').value),float(self.get_parameter('origin_y').value)))
  self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self);self.class_pub=self.create_publisher(OccupancyGrid,'/lunabot/semantic_map/class',1);self.conf_pub=self.create_publisher(OccupancyGrid,'/lunabot/semantic_map/confidence',1)
  topics=[str(self.get_parameter(name).value) for name in ('class_topic','confidence_topic','depth_topic','camera_info_topic')];subs=[Subscriber(self,Image,topics[0]),Subscriber(self,Image,topics[1]),Subscriber(self,Image,topics[2]),Subscriber(self,CameraInfo,topics[3])]
  self.sync=ApproximateTimeSynchronizer(subs,10,float(self.get_parameter('sync_slop_s').value));self.sync.registerCallback(self.callback)
 def callback(self,class_msg,confidence_msg,depth_msg,info):
  try:transform=self.tf_buffer.lookup_transform(str(self.get_parameter('map_frame').value),depth_msg.header.frame_id,depth_msg.header.stamp,timeout=Duration(seconds=.1))
  except TransformException as exc:self.get_logger().warning(f'TF unavailable at image stamp: {exc}');return
  labels=self.bridge.imgmsg_to_cv2(class_msg,'mono8');confidence=self.bridge.imgmsg_to_cv2(confidence_msg,'32FC1');depth=self.bridge.imgmsg_to_cv2(depth_msg,'32FC1');height,width=labels.shape
  rows=(np.arange(height)*depth.shape[0]/height).astype(int).clip(0,depth.shape[0]-1);cols=(np.arange(width)*depth.shape[1]/width).astype(int).clip(0,depth.shape[1]-1);depth=depth[rows[:,None],cols[None,:]]
  intrinsics={'fx':info.k[0]*width/info.width,'fy':info.k[4]*height/info.height,'cx':info.k[2]*width/info.width,'cy':info.k[5]*height/info.height};matrix=matrix_from_transform(transform.transform);stamp=depth_msg.header.stamp.sec*1000000000+depth_msg.header.stamp.nanosec;stride=int(self.get_parameter('sample_stride').value)
  for v in range(0,height,stride):
   for u in range(0,width,stride):
    label=int(labels[v,u]);value=float(depth[v,u]);certainty=float(confidence[v,u])
    if label and math.isfinite(value) and value>0:
     x,y,z=project_pixel(u,v,value,intrinsics,matrix);self.grid.observe(x,y,z,label,certainty,stamp)
  self.publish(depth_msg.header)
 def publish(self,header):
  layers=self.grid.dense_layers(header.stamp.sec*1000000000+header.stamp.nanosec)
  for publisher,name in ((self.class_pub,'semantic_class'),(self.conf_pub,'semantic_confidence')):
   msg=OccupancyGrid();msg.header=header;msg.header.frame_id=str(self.get_parameter('map_frame').value);msg.info.resolution=self.grid.resolution;msg.info.width=self.grid.width;msg.info.height=self.grid.height;msg.info.origin.position.x=self.grid.origin[0];msg.info.origin.position.y=self.grid.origin[1];msg.info.origin.orientation.w=1.0
   msg.data=[(-1 if value==0 else int(value)) for value in layers[name]] if name=='semantic_class' else [int(max(0,min(100,round(value*100)))) for value in layers[name]];publisher.publish(msg)
def main(args=None):
 rclpy.init(args=args);node=SemanticFusionNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
