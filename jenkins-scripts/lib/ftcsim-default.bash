#!/bin/bash -x
set -e

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
apt-get install -y python pkg-config cmake build-essential libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev libtinyxml-dev cppcheck gazebo ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-image-pipeline ros-${ROS_DISTRO}-joystick-drivers ros-${ROS_DISTRO}-rosbuild ros-${ROS_DISTRO}-rosunit ros-${ROS_DISTRO}-roslaunch

# Step 2: configure and build
rosdep init 
# Hack for not failing when github is down
update_done=false
seconds_waiting=0
while (! \$update_done); do
  rosdep update && update_done=true
  sleep 1
  seconds_waiting=$((seconds_waiting+1))
  [ \$seconds_waiting -gt 60 ] && exit 1
done

. /opt/ros/${ROS_DISTRO}/setup.sh
. /usr/share/gazebo/setup.sh

rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=$WORKSPACE/install $WORKSPACE/ftcsim
make -j${MAKE_JOBS}
make install
SHELL=/bin/sh . $WORKSPACE/install/share/ftcsim/setup.sh
export PATH="\$PATH:$WORKSPACE/install/bin/"
make test ARGS="-VV" || true

# Step 3: code check
cd $WORKSPACE/ftcsim
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
cat $WORKSPACE/build/cppcheck_results/*.xml
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
