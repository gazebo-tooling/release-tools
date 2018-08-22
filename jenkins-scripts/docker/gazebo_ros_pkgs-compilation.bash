#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Handle github-pullrequest jenkins plugin if present
if [[ -n ${ghprbTargetBranch} ]]; then
  if [[ ${ghprbTargetBranch} != 'ros2' ]]; then
    export ROS_DISTRO="${ghprbTargetBranch/-devel}"
  fi
fi

if [[ -z ${ARCH} ]]; then
  echo "ARCH variable not set!"
  exit 1
fi

if [[ -z ${DISTRO} ]]; then
  echo "DISTRO variable not set!"
  exit 1
fi

if [[ -z ${ROS_DISTRO} ]]; then
  echo "ROS_DISTRO variable not set!"
  exit 1
fi

. ${SCRIPT_DIR}/lib/gazebo_ros_pkgs-base.bash
