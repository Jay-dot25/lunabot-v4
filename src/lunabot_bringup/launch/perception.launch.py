"""Select heuristic, ML, or evaluation-only ground-truth perception."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
 mode=LaunchConfiguration('perception_mode')
 return LaunchDescription([
  DeclareLaunchArgument('perception_mode',default_value='heuristic',choices=['heuristic','ml','ground_truth']),
  DeclareLaunchArgument('model_path',default_value=''),DeclareLaunchArgument('model_config',default_value=''),DeclareLaunchArgument('model_checksum',default_value=''),DeclareLaunchArgument('inference_device',default_value='auto'),
  LogInfo(condition=LaunchConfigurationEquals('perception_mode','heuristic'),msg='Using legacy heuristic terrain perception.'),
  LogInfo(condition=LaunchConfigurationEquals('perception_mode','ground_truth'),msg='WARNING: ground_truth perception is evaluation-only and forbidden for final navigation results.'),
  Node(condition=LaunchConfigurationEquals('perception_mode','ml'),package='lunabot_perception',executable='terrain_inference_node',name='terrain_inference',output='screen',parameters=[{'model_path':LaunchConfiguration('model_path'),'config_path':LaunchConfiguration('model_config'),'checksum_path':LaunchConfiguration('model_checksum'),'device':LaunchConfiguration('inference_device')}]),
 ])
