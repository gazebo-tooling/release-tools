#!/bin/bash -x

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get ROS repo's key, to be used both in installing prereqs here and in creating the pbuilder chroot
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
# OSRF repository to get bullet
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need. Note the different curl provider package
apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} liblapack-dev

# Step 2: build and install simbody
# svn co https://simtk.org/svn/simbody/branches/Simbody3.0.1 ~/simbody
svn co https://simtk.org/svn/simbody/trunk ~/simbody
cd ~/simbody
mkdir build
cd build
cmake -DSimTK_INSTALL_PREFIX=/usr ..
make install

# Step 3: configure and build

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake -DPKG_CONFIG_PATH=/opt/ros/fuerte/lib/pkgconfig:/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib/pkgconfig -DCMAKE_INSTALL_PREFIX=$WORKSPACE/install -DSimTK_INSTALL_PREFIX=/usr -DCMAKE_MODULE_PATH=/usr/share/cmake $WORKSPACE/gazebo
make -j3
make install
. $WORKSPACE/install/share/gazebo-1.*/setup.sh
LD_LIBRARY_PATH=/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib make test ARGS="-VV" || true

# Step 3: code check
cd $WORKSPACE/gazebo
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

