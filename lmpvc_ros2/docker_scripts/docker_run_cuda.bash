#!/bin/bash
sudo docker run -d -t --rm --runtime=nvidia --gpus all \
     --env PULSE_SERVER=tcp:127.0.0.1:5050 \
     --env TZ="Europe/Helsinki" \
     --net=host \
     --ipc=host \
     --pid=host \
     --name lmpvc_ros2 \
     --volume "$PWD":/lmpvc_ros2_ws \
     --volume /etc/localtime:/etc/localtime:ro \
     lmpvc_ros2_cuda

echo "Attempting to start Pulseaudio TCP server..."
echo "Note: Will report "Failure" if server is already running."
pactl load-module module-native-protocol-tcp port=5050 auth-ip-acl=127.0.0.1