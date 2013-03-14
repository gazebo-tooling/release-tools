#!/bin/bash -x
set -e

# Use always DISPLAY in drcsim project
export DISPLAY=$(ps aux | grep "X :" | grep -v grep | awk '{ print $12 }')

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

if [ -z ${GZ_BUILD_TYPE} ]; then
    GZ_CMAKE_BUILD_TYPE=
else
    GZ_CMAKE_BUILD_TYPE="-DCMAKE_BUILD_TYPE=${GZ_BUILD_TYPE}"
fi

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

# Install drcsim's and gazebo Build-Depends
apt-get install -y cmake debhelper ros-${ROS_DISTRO}-pr2-mechanism ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-image-common ros-${ROS_DISTRO}-geometry ros-${ROS_DISTRO}-pr2-controllers ros-${ROS_DISTRO}-geometry-experimental ros-${ROS_DISTRO}-image-pipeline build-essential  libfreeimage-dev libprotoc-dev libprotobuf-dev protobuf-compiler freeglut3-dev libcurl4-openssl-dev libtinyxml-dev libtar-dev libtbb-dev libogre-dev libbullet-dev ros-${ROS_DISTRO}-visualization-common libxml2-dev pkg-config libqt4-dev ros-${ROS_DISTRO}-urdfdom ros-${ROS_DISTRO}-console-bridge libltdl-dev libboost-thread-dev libboost-signals-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev cppcheck ros-${ROS_DISTRO}-robot-model-visualization osrf-common sandia-hand

# Optional stuff. Check for graphic card support
if ${GRAPHIC_CARD_FOUND}; then
    apt-get install -y ${GRAPHIC_CARD_PKG}
fi

# Normal cmake routine for Gazebo
apt-get install -y mercurial ca-certificates
rm -fr $WORKSPACE/gazebo
hg clone https://bitbucket.org/osrf/gazebo $WORKSPACE/gazebo

rm -rf $WORKSPACE/gazebo/build $WORKSPACE/gazebo/install
mkdir -p $WORKSPACE/gazebo/build $WORKSPACE/gazebo/install
cd $WORKSPACE/gazebo/build
CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} cmake ${GZ_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/gazebo
make -j3
make install
. /usr/share/gazebo-1.*/setup.sh

# Step 3: configure and build drcim
if [ $DISTRO = quantal ]; then
    rosdep init && rosdep update
fi
# Normal cmake routine
. /opt/ros/${ROS_DISTRO}/setup.sh
. /usr/share/gazebo/setup.sh
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake ${GZ_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=$WORKSPACE/install $WORKSPACE/drcsim
make -j3
make install
SHELL=/bin/sh . $WORKSPACE/install/share/drcsim/setup.sh
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV" || true
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results rosrun rosunit clean_junit_xml.py
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
