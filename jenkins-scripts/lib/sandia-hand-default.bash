#!/bin/bash -x
set -e

if [ -z ${DISTRO} ]; then
    DISTRO=precise
fi

if [ -z ${ROS_DISTRO} ]; then
  ROS_DISTRO=fuerte
fi

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get ROS repo's key, to be used both in installing prereqs here and in creating the pbuilder chroot
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Required stuff for Gazebo
apt-get install -y cmake build-essential debhelper ros-fuerte-xacro ros-fuerte-ros osrf-common libboost-dev ros-fuerte-image-common ros-fuerte-ros-comm ros-fuerte-common-msgs
. /opt/ros/${ROS_DISTRO}/setup.sh

# Step 2: configure and build
# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE
export ROS_PACKAGE_PATH=`pwd`:/usr/share/osrf-common-1.0/ros:\$ROS_PACKAGE_PATH
cd $WORKSPACE/build
CMAKE_PREFIX_PATH=/opt/ros/fuerte cmake -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/sandia-hand
make -j3
make install
LD_LIBRARY_PATH=/opt/ros/fuerte/lib make test ARGS="-VV" || true
DELIM

# Make project-specific changes here
###################################################

sudo $WORKSPACE/pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
