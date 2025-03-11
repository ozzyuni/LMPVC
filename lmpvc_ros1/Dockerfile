FROM ubuntu:20.04

# ROS Noetic

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y \
    curl \
    software-properties-common

RUN add-apt-repository restricted && \
    add-apt-repository universe && \
    add-apt-repository multiverse

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
     	
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

RUN mkdir -p /lmpvc_ros1_ws/src

WORKDIR /lmpvc_ros1_ws

# This is intended as a temporary copy for use with rosdep.
# Typical usage is to override it with a bind mount later, as is done in docker_run.bash.
COPY ./src /lmpvc_ros1_ws/src

RUN rosdep init && rosdep update

RUN rosdep install --from-paths src -y --ignore-src
