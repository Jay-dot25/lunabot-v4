"""Phase 17 stage 2: verified Gazebo stage plus ROS sensor/command bridges."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('lunabot_bringup'), 'launch',
            'phase17_gazebo.launch.py',
        ]))
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='phase17_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lunabot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lunabot/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lunabot/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lunabot/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lunabot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/lunabot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/lunabot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )
    return LaunchDescription([gazebo, bridge])
