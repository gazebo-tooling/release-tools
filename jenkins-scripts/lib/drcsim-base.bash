#!/bin/bash -x

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get ROS repo's key
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Check for special gazebo versions.
# This line takes care of inside/outside variables importing GAZEBO_DEB_PACKAGE
# to build.sh scope
GAZEBO_DEB_PACKAGE=$GAZEBO_DEB_PACKAGE
if [ -z \$GAZEBO_DEB_PACKAGE ]; then
    GAZEBO_DEB_PACKAGE=gazebo
fi;

# Install drcsim's Build-Depends
apt-get install -y cmake debhelper ros-${ROS_DISTRO}-pr2-mechanism ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-image-common ros-${ROS_DISTRO}-geometry ros-${ROS_DISTRO}-pr2-controllers ros-${ROS_DISTRO}-geometry-experimental ros-${ROS_DISTRO}-robot-model-visualization ros-${ROS_DISTRO}-image-pipeline ros-${ROS_DISTRO}-console-bridge osrf-common sandia-hand \$GAZEBO_DEB_PACKAGE

# Step 2: configure and build

# Normal cmake routine
. /opt/ros/${ROS_DISTRO}/setup.sh
. /usr/share/gazebo/setup.sh
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=$WORKSPACE/install $WORKSPACE/drcsim
make -j3
make install
SHELL=/bin/sh . $WORKSPACE/install/share/drcsim/setup.sh
DISPLAY=:0 ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV" || true
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results rosrun rosunit clean_junit_xml.py

# Step 3: code check
CODE_CHECK_APP=$WORKSPACE/drcsim/tools/code_check.sh
if [ -f \$CODE_CHECK_APP ]; then
   cd $WORKSPACE/drcsim
   sh \$CODE_CHECK_APP -xmldir $WORKSPACE/build/cppcheck_results || true
fi
DELIM

# Make project-specific changes here
###################################################

sudo $WORKSPACE/pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

