ARG ROS_DISTRO=jazzy
ARG GAZEBO_DISTRO=harmonic
ARG URL_OSRF_ROSDEP_REPLACE=

FROM ros:$ROS_DISTRO-ros-base
ARG GAZEBO_DISTRO
ARG URL_OSRF_ROSDEP_REPLACE

ENV LANG C
ENV LC_ALL C
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
      coreutils \
      dpkg-dev \
      python3-bloom \
      wget \
    && rm -rf /var/lib/apt/lists/*

# Install osrf-rosdep for Gazebo distributions not in ROS.
# Note that 01 is important to give priority to the replace list below
RUN sudo bash -c \
  'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/01-gazebo.list'
# Bloom does not support environment vars, use the trick or replacing one rosdep keys by others
RUN sudo bash -c \
  "wget $URL_OSRF_ROSDEP_REPLACE -O /etc/ros/rosdep/sources.list.d/00-replace-gz.list"

# Not for using bloom but useful for rosdep calls
ENV GZ_VERSION=$GAZEBO_DISTRO
ENV IGNITION_VERSION=$GAZEBO_DISTRO

WORKDIR /tmp
COPY _check_metadata.bash .
COPY _rosdep_wrapper.bash .
ENTRYPOINT ["/tmp/_rosdep_wrapper.bash"]
