#!/bin/bash -x
set -e

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

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
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Install mercurial and drcsim's and gazebo Build-Depends
apt-get install -y git mercurial ca-certificates ${BASE_DEPENDENCIES} ${DRCSIM_BASE_DEPENDENCIES} ${ROS_GAZEBO_PKGS_DEPENDENCIES} ${GAZEBO_DEB_PACKAGE} ros-${ROS_DISTRO}-ros

# Optional stuff. Check for graphic card support
if ${GRAPHIC_CARD_FOUND}; then
    apt-get install -y ${GRAPHIC_CARD_PKG}
    # Check to be sure version of kernel graphic card support is the same.
    # It will kill DRI otherwise
    CHROOT_GRAPHIC_CARD_PKG_VERSION=\$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print \$3 }' | sed 's:-.*::')
    if [ "\${CHROOT_GRAPHIC_CARD_PKG_VERSION}" != "${GRAPHIC_CARD_PKG_VERSION}" ]; then
       echo "Package ${GRAPHIC_CARD_PKG} has different version in chroot and host system. Maybe you need to update your host" 
       exit 1
    fi
fi

. /opt/ros/${ROS_DISTRO}/setup.sh
. /usr/share/gazebo/setup.sh

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

# Create the catkin workspace
rm -fr $WORKSPACE/ws/src
mkdir -p $WORKSPACE/ws/src
cd $WORKSPACE/ws/src
catkin_init_workspace
hg clone $WORKSPACE/drcsim drcsim 
hg clone https://bitbucket.org/osrf/osrf-common osrf-common
hg clone https://bitbucket.org/osrf/sandia-hand sandia-hand
git clone https://github.com/ros-simulation/gazebo_ros_pkgs gazebo_ros_pkgs -b ${ROS_DISTRO}-devel

cd $WORKSPACE/ws
catkin_make -j${MAKE_JOBS} install
catkin_make_isolated -j${MAKE_JOBS} --install

# Testing procedure
SHELL=/bin/sh . $WORKSPACE/ws/install/setup.sh
SHELL=/bin/sh . $WORKSPACE/ws/install/share/drcsim/setup.sh

cd $WORKSPACE/ws/build
#ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV" || true
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV -R \\(atlas_publishers_hz_gpu.test\\|atlas_sandia_hands_publishers_hz_gpu.test\\|atlas_rosapi.test\\|atlas_sandia_hands_rosapi.test\\|vrc_task_1_scoring.test\\|vrc_task_1_gzlog_stop.test\\|vrc_task_1_dynamic_walking.test\\|multicamera_connection.test\\|vrc_final_task1_start_standup.test\\|vrc_final_task1_atlas_pubs_gpu.test\\)" || true
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results rosrun rosunit clean_junit_xml.py
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
