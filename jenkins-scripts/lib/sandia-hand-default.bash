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
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

apt-get install -y ${BASE_DEPENDENCIES} ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-ros osrf-common libboost-dev ros-${ROS_DISTRO}-image-common ros-${ROS_DISTRO}-ros-comm ros-${ROS_DISTRO}-common-msgs libqt4-dev

if [ $DISTRO = quantal ]; then
    rosdep init && rosdep update
fi

# Step 2: configure and build
# Normal cmake routine for sandia-hand
. /opt/ros/${ROS_DISTRO}/setup.sh
cd $WORKSPACE/sandia-hand
export ROS_PACKAGE_PATH=\$PWD:/usr/share/osrf-common-1.0/ros:\$ROS_PACKAGE_PATH
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
ls $WORKSPACE
CMAKE_PREFIX_PATH=/usr/share:/opt/ros/${ROS_DISTRO} cmake -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/sandia-hand
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
