#!/bin/bash -x

case ${DISTRO} in
  'trusty')
    ROS_DISTRO=indigo
    PKG_VERSION="7"
    ;;
  'xenial')
    ROS_DISTRO=kinetic
    PKG_VERSION="" # xenial uses 7 by default
    ;;
  *)
    echo "Unsupported DISTRO: ${DISTRO}"
    exit 1
esac

export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="srcsim_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Import srcim library
. ${SCRIPT_DIR}/lib/_srcsim_lib.bash

export ROS_SETUP_PREINSTALL_HOOK="""
${SRCSIM_SETUP_REPOSITORIES}
# Needed for rosdep
apt-get install -y ros-${ROS_DISTRO}-ros-base ${ROS_CATKIN_BASE}
# Need to get all srcsim dependencies (from package) No source pkg so
# can not use build-dep
apt-get install -y srcsim && apt-get remove -y srcsim
"""

export ROS_SETUP_POSTINSTALL_HOOK="""
${SRCSIM_INIT_SETUP}
${SRCSIM_ENV_SETUP}
"""

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "srcsim"

# don't have rosdep at this point and want gazebo to be cached by docker
DEPENDENCY_PKGS="libgazebo${PKG_VERSION}-dev"
# ROS packages come from the mirror in the own SRCSim repository
USE_ROS_REPO=false
OSRF_REPOS_TO_USE="stable"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
