"""GPS-denied EKF launch; ground truth is never an estimator input."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
 config=os.path.join(get_package_share_directory('lunabot_localization'),'config','ekf.yaml')
 return LaunchDescription([Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',output='screen',parameters=[config],remappings=[('odometry/filtered','/odometry/filtered')])])
