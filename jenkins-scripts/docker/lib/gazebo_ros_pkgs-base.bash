#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

DOCKER_JOB_NAME="gazebo_ros_pkgs_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
. ${SCRIPT_DIR}/lib/_gazebo_utils.sh

# Generate the first part of the build.sh file for ROS
CATKIN_EXTRA_ARGS="--cmake-args -DENABLE_DISPLAY_TESTS:BOOL=ON"

ROS_SETUP_PREINSTALL_HOOK="""
${GAZEBO_MODEL_INSTALLATION}
"""

. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "gazebo_ros_pkgs"

# don't have rosdep at this point and want gazebo to be cached by docker
DEPENDENCY_PKGS="${ROS_GAZEBO_PKGS_DEPENDENCIES} ${_GZ_ROS_PACKAGES} wget"
USE_ROS_REPO=true

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
