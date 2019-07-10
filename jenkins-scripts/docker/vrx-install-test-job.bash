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

. ${SCRIPT_DIR}/lib/generic-install-base.bash
