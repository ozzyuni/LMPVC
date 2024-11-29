# LMPVC Release Repository

Language Model Program Voice Control (LMPVC) is a voice control framework for robotics, powered by Large Language Models (LLMs). More READMEs detailing the repository may be found within the subfolders.

## lmpvc_ros2
This is the main component of LMPVC, targeting ROS2. However, it is currently missing components MoveIt 2 and gripper control, because the hardware used for development only works in ROS1.

## lmpvc_ros1
This workspace contains resources for operating robots which only have support for ROS1, primarily Franka Emika Panda. This is where robot control is currently done.