from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)

from launch.substitutions import (
    Command,
    FindExecutable,
)

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml

def build_cmd(params, prefix = ""):
    cmd = []
    for name, value in params.items():
        if isinstance(value, dict):
            cmd += build_cmd(value, prefix = prefix + name + '.')
        else:
            cmd.append('-p')    
            cmd.append(prefix + name + ':=' + str(value))
    
    return cmd

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
        'lmpvc_fr3_moveit_config', 'config/kinematics.yaml'
    )

    # Converting to the cmd fornat required by ExecuteProcess
    kinematics_params = build_cmd(kinematics_yaml)

    # Starting with ExecuteProcess instead of Node to avoid having multiple nodes with the same name
    lmpvc_process = ExecuteProcess(
        cmd=['ros2', 'run', 'lmpvc_controller', 'controller', '--ros-args',
             '-p', 'gripper_enabled:=True',
             '-p', 'gripper_plugin:="lmpvc_gripper_franka_plugins::FrankaHand"',
             '-p', 'planning_group_name:="fr3_manipulator"'
            ] + kinematics_params,
        output='screen'
    )

    return LaunchDescription([lmpvc_process])