from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lmpvc_listener',
            executable='server',
            name='listener'
        ),
        Node(
            package='lmpvc_talker',
            executable='service',
            name='talker'
        ),
    ])