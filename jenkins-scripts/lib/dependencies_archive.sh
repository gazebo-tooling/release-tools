#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes


if [[ -z $ROS_DISTRO ]]; then
    echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
    exit 1
fi

# Dart flags. Enable it by default unless compiled from source
DART_FROM_PKGS=false

if [ -z ${DART_COMPILE_FROM_SOURCE} ]; then
   DART_COMPILE_FROM_SOURCE=false
fi

if ${DART_COMPILE_FROM_SOURCE}; then
    DART_FROM_PKGS=false
fi

if $DART_FROM_PKGS; then
    if $DART_USE_4_VERSION; then
       dart_pkg="libdart-core4-dev"
    else
       dart_pkg="libdart-core3-dev"
    fi
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
# ruby for xml_schemas generation and libxml2-utils for xmllint used in tests
SDFORMAT_BASE_DEPENDENCIES="python                       \\
                            libboost-system-dev          \\
                            libboost-filesystem-dev      \\
                            libboost-program-options-dev \\
                            libboost-regex-dev           \\
                            libboost-iostreams-dev       \\
                            libtinyxml-dev               \\
                            ruby1.9.1-dev                \\
			    libxml2-utils"

# Need to explicit define to use old sdformat package
if [[ -z ${USE_OLD_SDFORMAT} ]]; then
    USE_OLD_SDFORMAT=false
fi

if ${USE_OLD_SDFORMAT}; then
    sdformat_pkg="sdformat"
else
    sdformat_pkg="libsdformat2-dev"
fi

# GAZEBO related dependencies

# Old versions used libogre-dev
ogre_pkg="libogre-1.8-dev"
if [[ ${DISTRO} == 'precise' ]] || \
   [[ ${DISTRO} == 'raring' ]] || \
   [[ ${DISTRO} == 'quantal' ]]; then
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
                          ${dart_pkg}                      \\
                          ${sdformat_pkg}"

GAZEBO_EXTRA_DEPENDENCIES="robot-player-dev \\
                           libcegui-mk2-dev \\
                           libavformat-dev  \\
                           libavcodec-dev   \\
                           libswscale-dev   \\
                           ruby-ronn"

GAZEBO_DEB_PACKAGE=$GAZEBO_DEB_PACKAGE
if [ -z $GAZEBO_DEB_PACKAGE ]; then
    GAZEBO_DEB_PACKAGE=libgazebo4-dev
fi

#
# DRCSIM_DEPENDENCIES
#
# image-transport-plugins is needed to properly advertise compressed image topics
DRCSIM_BASE_DEPENDENCIES="ros-${ROS_DISTRO}-std-msgs                          \\
                          ros-${ROS_DISTRO}-common-msgs                       \\
                          ros-${ROS_DISTRO}-image-common                      \\
                          ros-${ROS_DISTRO}-geometry                          \\
                          ros-${ROS_DISTRO}-geometry-experimental             \\
                          ros-${ROS_DISTRO}-image-pipeline                    \\
                          ros-${ROS_DISTRO}-image-transport-plugins           \\
                          ros-${ROS_DISTRO}-gazebo4-plugins                   \\
                          ros-${ROS_DISTRO}-compressed-depth-image-transport  \\
                          ros-${ROS_DISTRO}-compressed-image-transport        \\
                          ros-${ROS_DISTRO}-theora-image-transport            \\
                          ros-${ROS_DISTRO}-control-msgs                      \\
                          ros-${ROS_DISTRO}-robot-model                       \\
                          ros-${ROS_DISTRO}-control-toolbox                   \\
                          ${GAZEBO_DEB_PACKAGE}"

if [[ $ROS_DISTRO == 'hydro' ]]; then			  
  DRCSIM_BASE_DEPENDENCIES="${DRCSIM_BASE_DEPENDENCIES}          \\
                            ros-${ROS_DISTRO}-pr2-controllers    \\
                            ros-${ROS_DISTRO}-pr2-mechanism"
else
  DRCSIM_BASE_DEPENDENCIES="${DRCSIM_BASE_DEPENDENCIES}          \\
                            ros-${ROS_DISTRO}-controller-manager \\
                            ros-${ROS_DISTRO}-pr2-mechanism-msgs"
fi

# DRCSIM_FULL_DEPENDENCIES
# Need ROS postfix in precise for groovy/hydro 
if [[ $DISTRO == 'precise' ]]; then
   ROS_POSTFIX="-${ROS_DISTRO}"
else
   ROS_POSTFIX=""
fi

DRCSIM_FULL_DEPENDENCIES="${DRCSIM_BASE_DEPENDENCIES}       \\
                          sandia-hand${ROS_POSTFIX}         \\
    	                  osrf-common${ROS_POSTFIX}         \\
                          ros-${ROS_DISTRO}-gazebo4-plugins \\
                          ros-${ROS_DISTRO}-gazebo4-ros     \\
                          ${GAZEBO_DEB_PACKAGE}"
#
# SANDIA_HAND DEPENDECIES
#
SANDIA_HAND_BASE_DEPENDENCIES="ros-${ROS_DISTRO}-xacro              \\
                               ros-${ROS_DISTRO}-ros                \\
			       ros-${ROS_DISTRO}-image-common       \\
			       ros-${ROS_DISTRO}-ros-comm           \\
			       ros-${ROS_DISTRO}-common-msgs        \\
			       ros-${ROS_DISTRO}-message-generation \\
			       libboost-dev                         \\
			       libqt4-dev                           \\
			       osrf-common${ROS_POSTFIX}"

#			  
# ROS_GAZEBO_PKGS DEPENDECIES
#
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
			      ros-${ROS_DISTRO}-xacro                   \\
			      ros-${ROS_DISTRO}-cmake-modules"

if [[ $ROS_DISTRO != 'groovy' ]]; then
ROS_GAZEBO_PKGS_DEPENDENCIES="${ROS_GAZEBO_PKGS_DEPENDENCIES}           \\
                              ros-${ROS_DISTRO}-controller-manager      \\
                              ros-${ROS_DISTRO}-joint-limits-interface  \\
                              ros-${ROS_DISTRO}-transmission-interface"
fi

#
# DART dependencies
#
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

if ${DART_COMPILE_FROM_SOURCE}; then
    GAZEBO_EXTRA_DEPENDENCIES="$GAZEBO_EXTRA_DEPENDENCIES \\
                               $DART_DEPENDENCIES"
fi
