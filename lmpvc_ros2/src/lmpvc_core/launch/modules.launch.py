import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
  bringup_dir = get_package_share_directory('lmpvc_controller')
  launch_dir = os.path.join(bringup_dir, 'launch/fr3_lmpvc_controller.launch.py')
  
  return LaunchDescription([

    IncludeLaunchDescription(
    PythonLaunchDescriptionSource(launch_dir),
    ),
    Node(
      package='lmpvc_listener',
      executable='listener',
    ),

    Node(
      package='lmpvc_talker',
      executable='talker',
    ),

    Node(
      package='lmpvc_codegen',
      executable='codegen',
    ),

    Node(
      package='lmpvc_detector',
      executable='detector',
    ),

  ])