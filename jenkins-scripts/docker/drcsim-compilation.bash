#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

if [[ -z ${ARCH} ]]; then
  echo "ARCH variable not set!"
  exit 1
fi

if [[ -z ${DISTRO} ]]; then
  echo "DISTRO variable not set!"
  exit 1
fi

export GPU_SUPPORT_NEEDED=true

export GAZEBO_DEB_PACKAGE=${GAZEBO_DEB_PACKAGE:-"libgazebo4-dev"}
export BUILDING_SOFTWARE_DIRECTORY="drcsim"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="DRCSIM_FULL_DEPENDENCIES"
export BUILDING_JOB_REPOSITORIES="stable"
export USE_ROS_REPO=true

. ${SCRIPT_DIR}/lib/generic-building-base.bash
