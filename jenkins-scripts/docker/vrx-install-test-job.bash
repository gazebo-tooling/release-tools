#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

source ${SCRIPT_DIR}/lib/_vrx_lib.bash

export INSTALL_JOB_POSTINSTALL_HOOK="""
source /opt/ros/$ROS_DISTRO/setup.bash
${VRX_SMOKE_TEST}
"""

export USE_ROS_REPO=true
# OSRF repo is only used to workaround on issue 
# https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo
export OSRF_REPOS_TO_USE=stable

. ${SCRIPT_DIR}/lib/generic-install-base.bash
