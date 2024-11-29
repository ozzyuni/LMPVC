# LMPVC ROS1 Compatibility Resources

Language Model Program Voice Control (LMPVC) is a voice control framework for robotics, powered by Large Language Models (LLMs).

LMPVC is designed for ROS2. This workspace contains resources for operating robots which only have support for ROS1, primarily Franka Emika Panda.

## Instructions
Shortcut to set up the environment on the Robolab CV PC:
```
    ossi_ros1
```

Launch the components you need:
```
roslaunch lmpvc_ros1_compatibility controller_adapter.launch
roslaunch lmpvc_ros1_compatibility detector_adapter.launch
```

Launching controller_adapter spawns MoveIt controller and a grasp action client, accessible through the other end of the adapter in ROS2. Detector_adapter subscribes to our object detection system and similarly connects to ROS2.

If desired, it's possible to run ROS1 and ROS2 on different PCs. For this, change the target IP in the launch files, or start the components manually with the -ip flag.

Start a camera node with suitable settings for the object detection:
```
source ./camera.sh
```

See usage for manually adding and removing static objects:
```
rosrun lmpvc_ros1_compatibility pose_generator --help
```