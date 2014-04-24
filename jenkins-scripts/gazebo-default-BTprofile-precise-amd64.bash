#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISTRO=precise
export ROS_DISTRO=groovy

export GZ_BUILD_TYPE=Profile
export EXTRA_PACKAGES=libgoogle-perftools-dev

. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
