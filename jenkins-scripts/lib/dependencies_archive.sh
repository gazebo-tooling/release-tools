#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes


if [[ -z $ROS_DISTRO ]]; then
    echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
    exit 1
fi

# mesa-utils for dri checks and xsltproc for qtest->junit conversion
BASE_DEPENDENCIES="build-essential \\
                   cmake           \\
                   debhelper       \\
                   mesa-utils      \\
                   cppcheck        \\
                   xsltproc        \\
                   python"

# 1. SDFORMAT
SDFORMAT_BASE_DEPENDENCIES="python                       \\
                            libboost-system-dev          \\
                            libboost-filesystem-dev      \\
                            libboost-program-options-dev \\
                            libboost-regex-dev           \\
                            libboost-iostreams-dev       \\
                            libtinyxml-dev"

# Need to explicit define to use old sdformat package
if [[ -z ${USE_OLD_SDFORMAT} ]]; then
    USE_OLD_SDFORMAT=false
fi

if ${USE_OLD_SDFORMAT}; then
    sdformat_pkg="sdformat"
else
    sdformat_pkg="libsdformat-dev"
fi

# GAZEBO related dependencies

# In saucy we need to use 1.8 package to avoid conflicts with boost
if [[ ${DISTRO} == 'saucy' ]]; then
    ogre_pkg="libogre-1.8-dev"
else
    ogre_pkg="libogre-dev"
fi

GAZEBO_BASE_DEPENDENCIES="libfreeimage-dev                 \\
                          libprotoc-dev                    \\
                          libprotobuf-dev                  \\
                          protobuf-compiler                \\
                          freeglut3-dev                    \\
                          libcurl4-openssl-dev             \\
                          libtinyxml-dev                   \\
                          libtar-dev                       \\
                          libtbb-dev                       \\
                          ${ogre_pkg}                      \\
                          libxml2-dev                      \\
                          pkg-config                       \\
                          libqt4-dev                       \\
                          libltdl-dev                      \\
                          libboost-thread-dev              \\
                          libboost-signals-dev             \\
                          libboost-system-dev              \\
                          libboost-filesystem-dev          \\
                          libboost-program-options-dev     \\
                          libboost-regex-dev               \\
                          libboost-iostreams-dev           \\
                          libbullet2.82-dev                \\
                          libsimbody-dev                   \\
                          ${sdformat_pkg}"
if $ENABLE_DART; then
    GAZEBO_BASE_DEPENDENCIES="$GAZEBO_BASE_DEPENDENCIES \\
                             libdart-core3-dev"
fi

GAZEBO_EXTRA_DEPENDENCIES="robot-player-dev \\
                           libcegui-mk2-dev \\
                           libavformat-dev  \\
                           libavcodec-dev   \\
                           libswscale-dev   \\
                           ruby-ronn"

GAZEBO_DEB_PACKAGE=$GAZEBO_DEB_PACKAGE
if [ -z $GAZEBO_DEB_PACKAGE ]; then
    GAZEBO_DEB_PACKAGE=gazebo-current
fi

# image-transport-plugins is needed to properly advertise compressed image topics
DRCSIM_BASE_DEPENDENCIES="ros-${ROS_DISTRO}-pr2-mechanism                     \\
                          ros-${ROS_DISTRO}-std-msgs                          \\
                          ros-${ROS_DISTRO}-common-msgs                       \\
                          ros-${ROS_DISTRO}-image-common                      \\
                          ros-${ROS_DISTRO}-geometry                          \\
                          ros-${ROS_DISTRO}-pr2-controllers                   \\
                          ros-${ROS_DISTRO}-geometry-experimental             \\
                          ros-${ROS_DISTRO}-robot-model-visualization         \\
                          ros-${ROS_DISTRO}-image-pipeline                    \\
                          ros-${ROS_DISTRO}-image-transport-plugins           \\
                          ros-${ROS_DISTRO}-compressed-depth-image-transport  \\
                          ros-${ROS_DISTRO}-compressed-image-transport        \\
                          ros-${ROS_DISTRO}-theora-image-transport"

DRCSIM_FULL_DEPENDENCIES="${DRCSIM_BASE_DEPENDENCIES}      \\
                          sandia-hand                      \\
			  osrf-common                      \\
                          ros-${ROS_DISTRO}-gazebo-plugins \\
                          ros-${ROS_DISTRO}-gazebo-ros     \\
                          ${GAZEBO_DEB_PACKAGE}"

# ros-gazebo-pkgs dependencies
ROS_GAZEBO_PKGS_DEPENDENCIES="libtinyxml-dev                            \\
                              ros-${ROS_DISTRO}-catkin                  \\
			      ros-${ROS_DISTRO}-pluginlib               \\
			      ros-${ROS_DISTRO}-roscpp                  \\
			      ros-${ROS_DISTRO}-driver-base             \\
			      ros-${ROS_DISTRO}-angles                  \\
			      ros-${ROS_DISTRO}-cv-bridge               \\
			      ros-${ROS_DISTRO}-diagnostic-updater      \\
			      ros-${ROS_DISTRO}-dynamic-reconfigure     \\
			      ros-${ROS_DISTRO}-geometry-msgs           \\
			      ros-${ROS_DISTRO}-image-transport         \\
			      ros-${ROS_DISTRO}-message-generation      \\
			      ros-${ROS_DISTRO}-nav-msgs                \\
			      ros-${ROS_DISTRO}-nodelet                 \\
			      ros-${ROS_DISTRO}-pcl-conversions         \\
			      ros-${ROS_DISTRO}-pcl-ros                 \\
			      ros-${ROS_DISTRO}-polled-camera           \\
			      ros-${ROS_DISTRO}-rosconsole              \\
			      ros-${ROS_DISTRO}-rosgraph-msgs           \\
			      ros-${ROS_DISTRO}-sensor-msgs             \\
			      ros-${ROS_DISTRO}-std-srvs                \\
			      ros-${ROS_DISTRO}-tf                      \\
			      ros-${ROS_DISTRO}-trajectory-msgs         \\
			      ros-${ROS_DISTRO}-urdf                    \\
			      ros-${ROS_DISTRO}-cmake-modules"

if [[ $ROS_DISTRO != 'groovy' ]]; then
ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_DEPENDENCIES}           \\
                              ros-${ROS_DISTRO}-controller-manager      \\
                              ros-${ROS_DISTRO}-joint-limits-interface  \\
                              ros-${ROS_DISTRO}-transmission-interface"
fi

# DART dependencies
if [ -z ${DART_COMPILE_FROM_SOURCE} ]; then
    DART_COMPILE_FROM_SOURCE=false
fi

if [ -z ${DART_FROM_PKGS} ]; then
    DART_FROM_PKGS=false
fi

DART_DEPENDENCIES="libflann-dev            \\
                   libgtest-dev            \\
		   libeigen3-dev           \\
		   libassimp-dev           \\
		   freeglut3-dev           \\
		   libxi-dev               \\
		   libxmu-dev              \\
		   libtinyxml-dev          \\
		   libtinyxml2-dev         \\
		   libfcl-dev              \\
		   liburdfdom-dev          \\
		   libboost-system-dev     \\
		   libboost-filesystem-dev"
