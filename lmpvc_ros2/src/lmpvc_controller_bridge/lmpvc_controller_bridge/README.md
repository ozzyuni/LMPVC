# LMPVC Low Level Controller Module
This package contains a ROS2 service and a TCP bridge to connect with the controller in ROS1. In the future, a ROS2 based controller can simply be substituted behind the same service interface.

**Start the service**
```
ros2 run lmpvc_controller service
```

**Start the sevice with a different destination IP**

Optionally, an IP address can be specified if the ROS1 module is on a different PC in the same LAN.

```
ros2 run lmpvc_controller service -ip x.x.x.x
```