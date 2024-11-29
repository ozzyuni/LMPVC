from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lmpvc_codegen',
            executable='service',
            name='codegen'
        ),
    ])