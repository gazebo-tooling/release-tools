#!/bin/bash

if [[ -z $ROS_DISTRO ]]; then
    echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
    exit 1
fi

# mesa-utils for dri checks and xsltproc for qtest->junit conversion
BASE_DEPENDENCIES="build-essential cmake debhelper mesa-utils cppcheck xsltproc"

# GAZEBO related dependencies
GAZEBO_BASE_DEPENDENCIES="libfreeimage-dev libprotoc-dev libprotobuf-dev protobuf-compiler freeglut3-dev libcurl4-openssl-dev libtinyxml-dev libtar-dev libtbb-dev libogre-dev libxml2-dev pkg-config libqt4-dev ros-${ROS_DISTRO}-urdfdom ros-${ROS_DISTRO}-console-bridge libltdl-dev libboost-thread-dev libboost-signals-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev libbullet-dev"
GAZEBO_EXTRA_DEPENDENCIES="robot-player-dev libcegui-mk2-dev libavformat-dev libavcodec-dev libswscale-dev"

# DRCSIM related dependencies
# Check for special gazebo versions when builiding gazebo dependant software
GAZEBO_DEB_PACKAGE=$GAZEBO_DEB_PACKAGE
if [ -z $GAZEBO_DEB_PACKAGE ]; then
    GAZEBO_DEB_PACKAGE=gazebo-nightly
    echo "TODO: Using nightly gazebo until end of VRC"
fi

# image-transport-plugins is needed to properly advertise compressed image topics
DRCSIM_BASE_DEPENDENCIES="ros-${ROS_DISTRO}-pr2-mechanism ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-image-common ros-${ROS_DISTRO}-geometry ros-${ROS_DISTRO}-pr2-controllers ros-${ROS_DISTRO}-geometry-experimental ros-${ROS_DISTRO}-robot-model-visualization ros-${ROS_DISTRO}-image-pipeline ros-${ROS_DISTRO}-console-bridge ${GAZEBO_DEB_PACKAGE} ros-${ROS_DISTRO}-image-transport-plugins"

# Extra dependencies for groovy
if [[ ${ROS_DISTRO} == 'groovy' ]]; then
     DRCSIM_BASE_DEPENDENCIES="${DRCSIM_BASE_DEPENDENCIES} ros-groovy-compressed-depth-image-transport ros-groovy-compressed-image-transport ros-groovy-theora-image-transport"
fi

echo "TODO: Using nightly sandia and osrf-common packages until end of VRC"
DRCSIM_FULL_DEPENDENCIES="${DRCSIM_BASE_DEPENDENCIES} sandia-hand-nightly osrf-common-nightly"
