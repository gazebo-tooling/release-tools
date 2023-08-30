ARG ROS_DISTRO=humble
ARG GAZEBO_DISTRO=garden
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

# Install osrf-rosdep for Gazebo distributions not in ROS
RUN sudo bash -c \
  'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
# Replace url
# since it does not support arbitrary environment variables
RUN sudo bash -c \
  "wget $URL_OSRF_ROSDEP_REPLACE -O /etc/ros/rosdep/sources.list.d/00-replace-gz.list"

# Not for using bloom but useful for rosdep calls
ENV GZ_VERSION=$GAZEBO_DISTRO
ENV IGNITION_VERSION=$GAZEBO_DISTRO

WORKDIR /tmp
COPY _check_metadata.bash .
COPY _rosdep_wrapper.bash .
ENTRYPOINT ["/tmp/_rosdep_wrapper.bash"]
