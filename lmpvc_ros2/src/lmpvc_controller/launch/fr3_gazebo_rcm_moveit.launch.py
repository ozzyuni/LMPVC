from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import (
    DeclareLaunchArgument
)

from launch.substitutions import (
    Command,
    FindExecutable,
)

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    kinematics_yaml = load_yaml(
        'fr3_moveit_config', 'config/kinematics.yaml'
    )

    # RCM MoveIt wrapper node
    lmpvc_controller = Node(
        name="lmpvc_controller",
        package="lmpvc_controller",
        executable="controller",
        output="screen",
        parameters=[
            kinematics_yaml,
            {'use_sim_time': True},
            {'planning_group_name': 'fr3_manipulator'}
        ],
    )

    return LaunchDescription([lmpvc_controller])