#!/bin/bash -x
set -e

if [ -z ${SOFTWARE_UNDER_TEST} ]; then
    echo "Need to define the SOFTWARE_UNDER_TEST variable"
    exit 1
fi

export GPU_SUPPORT_NEEDED=true
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

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

# Step 1: install everything you need. Need DRCSIM_BASE_DEPENDENCIES to
# get proper test support. It could be refactor.
apt-get install -y ${SOFTWARE_UNDER_TEST}-nightly mesa-utils ${DRCSIM_BASE_DEPENDENCIES}

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

# Need rosdep view to run >=quantal tests
if [ $DISTRO != precise ]; then
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
