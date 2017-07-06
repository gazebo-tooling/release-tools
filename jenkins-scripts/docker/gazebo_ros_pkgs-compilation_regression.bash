#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Handle github-pullrequest jenkins plugin if present
if [[ -n ${ghprbTargetBranch} ]]; then
  export ROS_DISTRO="${ghprbTargetBranch/-devel}"
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

ROS_WS_PREBUILD_HOOK="""
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo -b ${ROS_DISTRO}-devel
"""
. ${SCRIPT_DIR}/lib/gazebo_ros_pkgs-base.bash
