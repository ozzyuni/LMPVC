from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lmpvc_controller',
            executable='service',
            name='controller'
        ),
        Node(
            package='lmpvc_detector',
            executable='service',
            name='detector'
        ),
    ])