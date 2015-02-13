#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISTRO=trusty
export ROS_DISTRO=indigo

. ${SCRIPT_DIR}/lib/drcsim-catkin_ws-gazebo_latest-base.bash
