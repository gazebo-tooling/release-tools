#!/bin/bash -x
set -e

if [ -z ${DISTRO} ]; then
    DISTRO=precise
fi

if [ -z ${ROS_DISTRO} ]; then
  ROS_DISTRO=groovy
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
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

apt-get install -y ${BASE_DEPENDENCIES} ${SANDIA_HAND_BASE_DEPENDENCIES}

if [ $DISTRO != precise ]; then
    rosdep init && rosdep update
fi

# Step 2: configure and build
# Normal cmake routine for sandia-hand
. /opt/ros/${ROS_DISTRO}/setup.sh
cd $WORKSPACE/sandia-hand
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} cmake -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/sandia-hand
make -j${MAKE_JOBS}
make install
LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib make test ARGS="-VV" || true
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
