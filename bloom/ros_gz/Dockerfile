ARG ROS_DISTRO=humble

FROM ros:$ROS_DISTRO-ros-base

ENV LANG C
ENV LC_ALL C
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
      coreutils \
      dpkg-dev \
      python3-bloom \
      wget \
    && rm -rf /var/lib/apt/lists/*

# Install osrf-rosdep for Garden
RUN sudo bash -c \
  'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
# Replace Fortress with Garden to make bloom to work
# since it does not support arbitrary environemnt variables
RUN sudo bash -c \
  'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/replace_fortress_with_garden/00-replace-gz-fortress-with-garden.list -O /etc/ros/rosdep/sources.list.d/00-replace-gz-fortress-with-garden.list'

# Not for using bloom but useful for rosdep calls
ENV GZ_VERSION=garden
ENV IGNITION_VERSION=garden

WORKDIR /tmp
COPY _check_metadata.bash .
COPY _rosdep_wrapper.bash .
ENTRYPOINT ["/tmp/_rosdep_wrapper.bash"]
