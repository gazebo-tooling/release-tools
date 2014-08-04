#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

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

# check for graphic card support
GRAPHIC_TESTS=false
if [ $GRAPHIC_CARD_NAME = Nvidia ] && [ $DISTRO = quantal ]; then
    GRAPHIC_TESTS=true

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
fi

# Check for proper ros wrappers depending on gazebo version
if [ $PACKAGE_ALIAS = 'gazebo-current' ]; then
  ROS_GAZEBO_PKGS="ros-$ROS_DISTRO-gazebo-msgs-current \
                   ros-$ROS_DISTRO-gazebo-plugins-current \
		   ros-$ROS_DISTRO-gazebo-ros-current \
		   ros-$ROS_DISTRO-gazebo-ros-pkgs-current"
else
  ROS_GAZEBO_PKGS="ros-$ROS_DISTRO-$PACKAGE_ALIAS-msgs \
                   ros-$ROS_DISTRO-$PACKAGE_ALIAS-plugins \
		   ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros \
		   ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros-pkgs"
fi

# Need -ros for rosrun
apt-get install -y \$ROS_GAZEBO_PKGS ros-$ROS_DISTRO-ros

# Step 2: configure and build
if [ $ROS_DISTRO != groovy ]; then
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

SHELL=/bin/sh . /opt/ros/${ROS_DISTRO}/setup.sh

# In our nvidia machines, run the test to launch altas
if \$GRAPHIC_TESTS; then
  timeout 180 roslaunch gazebo_ros shapes_world.launch
else
  timeout 180 rosrun gazebo_ros gazebo
fi

DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
