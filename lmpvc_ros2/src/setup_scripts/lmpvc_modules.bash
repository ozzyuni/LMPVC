#!/bin/bash
. ${PWD}/install/setup.bash
export HF_HOME=$PWD/models/
ros2 launch lmpvc_core modules.launch.py