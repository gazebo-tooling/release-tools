#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="gazebo_ros_pkgs_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash ""

cat > build.sh << DELIM_CHECKOUT
###################################################
# Make project-specific changes here
#
set -ex

git clone https://github.com/ros-simulation/gazebo_ros_demos ${WORKSPACE}/gazebo_ros_demos
DELIM_CHECKOUT

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "gazebo_ros_demos"

cat >> build.sh << DELIM
cd ${CATKIN_WS}/gazebo_ros_demos/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD
cd ${CATKIN_WS}

timeout --preserve-status 180 roslaunch rrbot_gazebo rrbot_world.launch headless:=true extra_gazebo_args:="--verbose"
if [ $? != 0 ]; then
  echo "Failure response in the launch command" 
  exit 1
fi

echo "180 testing seconds finished successfully"
DELIM

USE_ROS_REPO=${USE_ROS_REPO:-true}
OSRF_REPOS_TO_USE=${OSRF_REPOS_TO_USE:-stable}

# Let's try to install all the packages and check the example
ROS_GAZEBO_PKGS="ros-$ROS_DISTRO-$PACKAGE_ALIAS-msgs \
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-plugins \
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros \
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros-pkgs"

DEPENDENCY_PKGS="${ROS_GAZEBO_PKGS} ${ROS_GAZEBO_PKGS_EXAMPLE_DEPS} git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
