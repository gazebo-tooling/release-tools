#!/bin/bash -x
set -e

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get ROS repo's key, to be used both in installing prereqs here and in creating the pbuilder chroot
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list'
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Required stuff for Gazebo
apt-get install -y ${BASE_DEPENDENCIES} ros-${ROS_DISTRO}-ros ros-${ROS_DISTRO}-ros-comm
. /opt/ros/${ROS_DISTRO}/setup.sh

# For >=quantal, need to start rosdep base
if [ "${ROS_DISTRO}" != "groovy" ]; then
  rosdep init
  rosdep update
fi

# Step 2: configure and build
# Normal cmake routine for osrf-common
cd $WORKSPACE/osrf-common
export ROS_PACKAGE_PATH=\$PWD:\$ROS_PACKAGE_PATH
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
ls $WORKSPACE
CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} cmake -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/osrf-common
make -j3
make install
LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib make test ARGS="-VV" || true
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
