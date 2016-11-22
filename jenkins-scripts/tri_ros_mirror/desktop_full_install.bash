#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# TRI ROS Mirror common library
. ${SCRIPT_DIR}/_common_library.bash

# Isolated repo
export INSTALL_JOB_PKG="ros-indigo-desktop-full"
export INSTALL_JOB_REPOS=""
export USE_ROS_REPO=false

INSTALL_JOB_PREINSTALL_HOOK="""
# import the TRI ROS mirror

echo \"deb http://54.183.148.69:8080/${APLTY_PUBLISH_REPO_PREFIX} ${DISTRO} main\" >\\
                          /etc/apt/sources.list.d/tri_ros_mirror.list
sudo apt-get update
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="wget bc"

. ${SCRIPT_DIR}/../docker/lib/generic-install-base.bash
