FROM ros:humble-ros-base-jammy
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>

ENV LANG C
ENV LC_ALL C
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
      python3-bloom \
      wget \
    && rm -rf /var/lib/apt/lists/*

# Install osrf-rosdep for Garden
RUN sudo bash -c \
  'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
# Replace Fortress with Garden
RUN sudo bash -c \
  'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/jrivero/rename_humble/gz/replace_fortress_with_garden/00-replace-gz-fortress-with-garden.list -O /etc/ros/rosdep/sources.list.d/00-replace-gz-fortress-with-garden.list'
RUN rosdep update
RUN rosdep resolve gz-fortress | grep gz-garden

ENV GZ_VERSION=garden

