from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lmpvc_core',
            executable='voice_control',
            name='core'
        ),
    ])