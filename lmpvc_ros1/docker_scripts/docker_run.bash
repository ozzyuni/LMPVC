#!/bin/bash
sudo docker run -d -t --rm --network host --name lmpvc_ros1 -v "$PWD":/lmpvc_ros1_ws lmpvc_ros1
