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

case ${DISTRO} in
  'xenial')
    ROS_DISTRO=kinetic
    PKG_VERSION="8"
    ;;
  *)
    echo "Unsupported DISTRO: ${DISTRO}"
    exit 1
esac

export GPU_SUPPORT_NEEDED=true

## Gazebo installation and configuration info
## Used in _ros_setup_catkin_make_isolated_buildsh.bash
export USE_GZ_VERSION_ROSDEP=true
export GAZEBO_VERSION_FOR_ROS="$PKG_VERSION"

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="servicesim_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Import servicesim library (none currently)
#. ${SCRIPT_DIR}/lib/_servicesim_lib.bash

export ROS_SETUP_PREINSTALL_HOOK="""
#${SERVICESIM_SETUP_REPOSITORIES} # currently in progress
## Needed for rosdep
apt-get install -y ros-${ROS_DISTRO}-ros-base ${ROS_CATKIN_BASE}
## Need to get all servicesim dependencies (from package) No source pkg so
## can not use build-dep
#apt-get install -y servicesim && apt-get remove -y servicesim # currently in progress
"""

export ROS_SETUP_POSTINSTALL_HOOK="""
"""

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_catkin_make_isolated_buildsh.bash "servicesim"

# don't have rosdep at this point and want gazebo to be cached by docker
DEPENDENCY_PKGS="libgazebo${PKG_VERSION}-dev"
USE_ROS_REPO=true
OSRF_REPOS_TO_USE="stable"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
