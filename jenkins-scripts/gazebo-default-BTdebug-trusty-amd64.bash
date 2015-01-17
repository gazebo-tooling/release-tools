#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISTRO=trusty
export ROS_DISTRO=indigo

export GZ_BUILD_TYPE=Debug

. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
