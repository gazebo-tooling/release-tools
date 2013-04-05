#!/bin/bash -x
set -e

if [ -z ${SOFTWARE_UNDER_TEST} ]; then
    echo "Need to define the SOFTWARE_UNDER_TEST variable"
    exit 1
fi

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
apt-get install -y ${SOFTWARE_UNDER_TEST}-nightly

# Step 2: load all setup files available
if [ -f /opt/ros/${ROS_DISTRO}/setup.sh ]; then
  . /opt/ros/${ROS_DISTRO}/setup.sh
fi
if [ -f /usr/share/gazebo/setup.sh ]; then
  . /usr/share/gazebo/setup.sh
fi
if [ -f /usr/share/drcsim/setup.sh ]; then
  SHELL=/bin/sh . /usr/share/drcsim/setup.sh
fi

# Step 3: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/${SOFTWARE_UNDER_TEST}
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV" || true

if [ $SOFTWARE_UNDER_TEST = 'drcsim' ]; then
  ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results rosrun rosunit clean_junit_xml.py
fi

DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
