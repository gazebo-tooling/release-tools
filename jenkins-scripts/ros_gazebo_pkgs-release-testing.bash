#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# debbuild typically does not define ROS_DISTRO so autogenerate it
# bloom ros-gazebo-pkgs define ROS_DISTRO, ignore non supported drcsim 
if [[ -z $ROS_DISTRO ]]; then
    [[ $DISTRO = 'precise' ]] && export ROS_DISTRO=hydro
    [[ $DISTRO = 'trusty'  ]] && export ROS_DISTRO=indigo
fi

. ${SCRIPT_DIR}/lib/ros_gazebo_pkgs-check-release.bash
