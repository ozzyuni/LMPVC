# LMPVC Release Repository - IEEE Roman version (WIP)

Language Model Program Voice Control (LMPVC) is a voice control framework for robotics, powered by Large Language Models (LLMs). More READMEs detailing the repository may be found within the subfolders.

## Experiments

Videos about the complete assembly task and teaching of the associated Policies can be found in the links below:

https://youtu.be/SqL07pA2dIA
https://youtu.be/vfTIiDHBQ-4

Other experiment materials are collected in the *experiments* directory.

## lmpvc_ros2
This is the main component of LMPVC, targeting ROS2. However, it is currently missing components MoveIt 2 and gripper control, because the hardware used for development only works in ROS1.

## lmpvc_ros1
This workspace contains resources for operating robots which only have support for ROS1, primarily Franka Emika Panda. This is where robot control is currently done.
