"""Synchronized ROS 2 ONNX terrain inference with safe failure behavior."""
from __future__ import annotations
import json,time
from pathlib import Path
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer,Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo,Image
from std_msgs.msg import Header
from lunabot_msgs.msg import TerrainPrediction
from .inference_core import InferenceContractError,OnnxTerrainInference,load_contract

COLORS=np.asarray([[90,90,90],[194,178,128],[145,122,82],[130,150,170],[235,145,45],[220,45,35],[125,35,145],[35,45,85],[30,180,210]],dtype=np.uint8)

class TerrainInferenceNode(Node):
 def __init__(self):
  super().__init__('terrain_inference')
  defaults={'model_path':'','config_path':'','checksum_path':'','device':'auto','image_topic':'/lunabot/camera/image_raw','camera_info_topic':'/lunabot/camera/camera_info','class_topic':'/lunabot/terrain/class_image','confidence_topic':'/lunabot/terrain/confidence','overlay_topic':'/lunabot/terrain/overlay','status_topic':'/lunabot/terrain/inference_status','sync_slop_s':0.05,'queue_size':5}
  for key,value in defaults.items():self.declare_parameter(key,value)
  self.bridge=CvBridge();self.frames=0;self.dropped=0;self.started=time.monotonic();self.backend=None;self.error=''
  self.class_pub=self.create_publisher(Image,self._param('class_topic'),10);self.confidence_pub=self.create_publisher(Image,self._param('confidence_topic'),10);self.overlay_pub=self.create_publisher(Image,self._param('overlay_topic'),10);self.status_pub=self.create_publisher(TerrainPrediction,self._param('status_topic'),10)
  try:
   model,config=self._param('model_path'),self._param('config_path');checksum=self._param('checksum_path') or None
   contract=load_contract(model,config,checksum);self.backend=OnnxTerrainInference(model,contract,self._param('device'));self.model_name=Path(model).name
  except Exception as exc:
   self.error=str(exc);self.model_name='unavailable';self.get_logger().error(f'inference disabled safely: {self.error}')
  image=Subscriber(self,Image,self._param('image_topic'));info=Subscriber(self,CameraInfo,self._param('camera_info_topic'))
  self.sync=ApproximateTimeSynchronizer([image,info],queue_size=int(self.get_parameter('queue_size').value),slop=float(self.get_parameter('sync_slop_s').value));self.sync.registerCallback(self._callback)
  self.timer=self.create_timer(1.0,self._heartbeat)
 def _param(self,name):return str(self.get_parameter(name).value)
 def _status(self,header,state,detail,latency=0.0,class_image=None,confidence=None):
  msg=TerrainPrediction();msg.header=header;msg.state=state;msg.model_name=self.model_name;msg.model_version='onnx-schema-1';msg.inference_time_ms=float(latency);msg.frames_per_second=float(self.frames/max(1e-6,time.monotonic()-self.started));msg.dropped_frames=self.dropped;msg.detail=detail
  if class_image is not None:msg.class_image=class_image
  if confidence is not None:msg.confidence_image=confidence
  self.status_pub.publish(msg)
 def _heartbeat(self):
  if self.backend is None:
   header=Header();header.stamp=self.get_clock().now().to_msg();self._status(header,TerrainPrediction.STATE_ERROR,self.error)
 def _callback(self,image,info):
  if self.backend is None:self.dropped+=1;self._status(image.header,TerrainPrediction.STATE_ERROR,self.error);return
  try:
   rgb=self.bridge.imgmsg_to_cv2(image,desired_encoding='rgb8');labels,confidence,latency=self.backend.infer(rgb)
   class_msg=self.bridge.cv2_to_imgmsg(labels,encoding='mono8');conf_msg=self.bridge.cv2_to_imgmsg(confidence,encoding='32FC1');overlay_msg=self.bridge.cv2_to_imgmsg(COLORS[labels],encoding='rgb8')
   for output in (class_msg,conf_msg,overlay_msg):output.header=image.header
   self.class_pub.publish(class_msg);self.confidence_pub.publish(conf_msg);self.overlay_pub.publish(overlay_msg);self.frames+=1
   self._status(image.header,TerrainPrediction.STATE_READY,'ok',latency,class_msg,conf_msg)
  except Exception as exc:
   self.dropped+=1;self._status(image.header,TerrainPrediction.STATE_DEGRADED,str(exc));self.get_logger().error(f'inference frame dropped: {exc}')

def main(args=None):
 rclpy.init(args=args);node=TerrainInferenceNode()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
