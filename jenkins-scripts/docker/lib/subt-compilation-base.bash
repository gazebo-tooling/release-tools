#!/bin/bash -x

case ${DISTRO} in
  'xenial')
    ROS_DISTRO=kinetic
    GAZEBO_VERSION_FOR_ROS="9"
    ;;
  *)
    echo "Unsupported DISTRO: ${DISTRO}"
    exit 1
esac

export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="subt_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh


export ROS_SETUP_PREINSTALL_HOOK="""
mkdir -p /etc/ros/rosdep/sources.list.d/
wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo9/00-gazebo9.list -O /etc/ros/rosdep/sources.list.d/00-gazebo9.list
"""

export ROS_SETUP_POSTINSTALL_HOOK="""
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
mkdir -p ~/.gazebo/models
tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz
"""

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "subt"

DEPENDENCY_PKGS="${SUBT_DEPENDENCIES}"
# ROS packages come from the mirror in the own subt repository
USE_ROS_REPO=true
OSRF_REPOS_TO_USE="stable"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
