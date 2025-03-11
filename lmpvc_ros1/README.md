# LMPVC ROS1 Compatibility Resources

Language Model Program Voice Control (LMPVC) is a voice control framework for robotics, powered by Large Language Models (LLMs).

LMPVC is designed for ROS2. This workspace contains resources for operating robots which only have support for ROS1, primarily Franka Emika Panda.

## Instructions
Launch the components you need:
```
roslaunch lmpvc_ros1_compatibility controller_adapter.launch
roslaunch lmpvc_ros1_compatibility detector_adapter.launch
```

Launching controller\_adapter spawns MoveIt controller and a grasp action client, accessible through the other end of the adapter in ROS2. Detector\_adapter subscribes to our object detection system and similarly connects to ROS2.

If desired, it's possible to run ROS1 and ROS2 on different PCs. For this, change the target IP in the launch files, or start the components manually with the -ip flag.

Start a camera node with suitable settings for the object detection:
```
source ./camera.sh
```

See usage for manually adding and removing static objects:
```
rosrun lmpvc_ros1_compatibility pose_generator --help
```

## Docker
Using the recommended steps below, a Docker image called *lmpvc_ros1* will be created and run in a container called *lmpvc_ros1*. Networking is configure to host, meaning ROS will work as if it was running on the host PC, and this workspace will be mounted in /lmpvc\_ros1\_ws.

### IMPORTANT

Before proceeding, not that the scripts in *./docker_scripts* use *sudo* to run Docker commands.

### Recommended steps:

Build the image. This step is only necessary once, unless the image is removed.

```
./docker_scripts/docker_build.bash
```

Spin up the container. Mounts the working directory, so only run in workspace root.

```
./docker_scripts/docker_run.bash
```

Execute bash (run a terminal) inside the container.

```
./docker_scripts/docker_terminal.bash
```

You can now build the workspace and start using it!

Once done, stop the container. It was ran with the *--rm* option, meaning it will automatically be removed. Any changes will persist because of the bind mounted workspace.

```
./docker_scripts/docker_stop.bash
```
