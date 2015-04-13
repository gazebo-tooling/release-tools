#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes


if [[ -z $ROS_DISTRO ]]; then
    echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
    exit 1
fi

# Dart flags. Enable it by default unless compiled from source
if [ -z ${DART_COMPILE_FROM_SOURCE} ]; then
   DART_COMPILE_FROM_SOURCE=false
fi

if [ -z ${DART_FROM_PKGS} ]; then
  DART_FROM_PKGS=false
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
                            ruby1.9.1                    \\
			    libxml2-utils"

# Need to explicit define to use old sdformat package
if [[ -z ${USE_OLD_SDFORMAT} ]]; then
    USE_OLD_SDFORMAT=false
fi

if ${USE_OLD_SDFORMAT}; then
    sdformat_pkg="sdformat"
elif [[ ${GAZEBO_MAJOR_VERSION} -ge 6 ]]; then
    sdformat_pkg="libsdformat3-dev-prerelease"
else
    sdformat_pkg="libsdformat2-dev"
fi

# SDFORMAT related dependencies
if [[ -z ${SDFORMAT_MAJOR_VERSION} ]]; then
    SDFORMAT_MAJOR_VERSION=3
fi

if [[ ${SDFORMAT_MAJOR_VERSION} -ge 3 ]]; then
    # sdformat3 requires ignition-math
    SDFORMAT_BASE_DEPENDENCIES="${SDFORMAT_BASE_DEPENDENCIES}          \\
                                libignition-math-dev"
fi

# GAZEBO related dependencies
if [[ -z ${GAZEBO_MAJOR_VERSION} ]]; then
    GAZEBO_MAJOR_VERSION=5
fi

if [[ -z $GAZEBO_DEB_PACKAGE ]];then
    GAZEBO_DEB_PACKAGE=libgazebo${GAZEBO_MAJOR_VERSION}-dev
fi

# Old versions used libogre-dev
ogre_pkg="libogre-1.9-dev"
if [[ ${DISTRO} == 'precise' ]] || \
   [[ ${DISTRO} == 'raring' ]] || \
   [[ ${DISTRO} == 'quantal' ]]; then
    ogre_pkg="libogre-dev"
elif [[ ${DISTRO} == 'trusty' ]]; then
    # All versions of gazebo (including 5) are using the 
    # ogre-1.8-dev package to keep in sync with ROS rviz 
    ogre_pkg="libogre-1.8-dev"
elif [[ ${GAZEBO_MAJOR_VERSION} -le 4 ]]; then
    # Before gazebo5, ogre 1.9 was not supported
    ogre_pkg="libogre-1.8-dev"
fi

# Starting from utopic, we are using the bullet provided by ubuntu
bullet_pkg="libbullet-dev"
if [[ ${DISTRO} == 'precise' ]] || \
   [[ ${DISTRO} == 'trusty' ]]; then
    bullet_pkg="libbullet2.82-dev"
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
                          libgts-dev                       \\
                          libboost-thread-dev              \\
                          libboost-signals-dev             \\
                          libboost-system-dev              \\
                          libboost-filesystem-dev          \\
                          libboost-program-options-dev     \\
                          libboost-regex-dev               \\
                          libboost-iostreams-dev           \\
                          libignition-math-dev             \\
                          ${bullet_pkg}                    \\
                          libsimbody-dev                   \\
                          ${dart_pkg}                      \\
                          ${sdformat_pkg}"


GAZEBO_EXTRA_DEPENDENCIES="robot-player-dev \\
                           libavformat-dev  \\
                           libavcodec-dev   \\
                           libgraphviz-dev  \\
                           libswscale-dev   \\
                           ruby-ronn"
		       
# cegui is deprecated in gazebo 6
if [[ ${GAZEBO_MAJOR_VERSION} -le 6 ]]; then
    GAZEBO_EXTRA_DEPENDENCIES="${GAZEBO_EXTRA_DEPENDENCIES} \\
                               libcegui-mk2-dev"
fi

# gdal is not working on precise
# it was added in gazebo5, which does not support precise
if [[ ${DISTRO} != 'precise' ]]; then
    GAZEBO_EXTRA_DEPENDENCIES="${GAZEBO_EXTRA_DEPENDENCIES} \\
                               libgdal-dev"
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
                          ros-${ROS_DISTRO}-robot-state-publisher             \\
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
                          ros-${ROS_DISTRO}-laser-assembler \\
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
			      ros-${ROS_DISTRO}-cmake-modules           \\
                              ros-${ROS_DISTRO}-controller-manager      \\
                              ros-${ROS_DISTRO}-joint-limits-interface  \\
                              ros-${ROS_DISTRO}-transmission-interface"

# These dependencies are for testing the ros_gazebo_pkgs			      
ROS_GAZEBO_PKGS_EXAMPLE_DEPS="ros-${ROS_DISTRO}-xacro                   \\
                              ros-${ROS_DISTRO}-effort-controllers      \\
                              ros-${ROS_DISTRO}-joint-state-controller"

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

#
# IGNITION
#

IGN_TRANSPORT_DEPENDENCIES="pkg-config           \\
			    python               \\
			    ruby-ronn            \\
			    libprotoc-dev        \\
			    libprotobuf-dev      \\
			    protobuf-compiler    \\
			    uuid-dev             \\
			    libzmq3-dev          \\
			    libczmq-dev"

#
# HAPTIX
#
HAPTIX_COMM_DEPENDENCIES="pkg-config                \\
                          libignition-transport-dev \\
                          libboost-system-dev       \\
			  libprotoc-dev             \\
			  libprotobuf-dev           \\
			  protobuf-compiler         \\
                	  liboctave-dev"
