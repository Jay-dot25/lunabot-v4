"""Convert the selected GPS-denied odometry estimate to PoseStamped."""
import rclpy
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
class LocalizationPosePublisher(Node):
 def __init__(self):
  super().__init__('localization_pose_publisher');self.declare_parameter('input_topic','/odometry/filtered');topic=str(self.get_parameter('input_topic').value);self.pub=self.create_publisher(PoseStamped,'/lunabot/localization/pose',10);self.tf=TransformBroadcaster(self);self.create_subscription(Odometry,topic,self.callback,10)
 def callback(self,msg):
  pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose;self.pub.publish(pose)
  transform=TransformStamped();transform.header=msg.header;transform.header.frame_id=msg.header.frame_id or 'odom';transform.child_frame_id='chassis';transform.transform.translation.x=msg.pose.pose.position.x;transform.transform.translation.y=msg.pose.pose.position.y;transform.transform.translation.z=msg.pose.pose.position.z;transform.transform.rotation=msg.pose.pose.orientation;self.tf.sendTransform(transform)
def main(args=None):
 rclpy.init(args=args);node=LocalizationPosePublisher()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
