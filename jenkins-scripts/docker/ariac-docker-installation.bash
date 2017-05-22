#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Need for priviledge mode (docker in docker)
export GPU_SUPPORT_NEEDED=true

if [[ -z ${ROS_DISTRO} ]]; then
    echo "Need to define a ROS_DISTRO value"
    exit
fi

INSTALL_JOB_PREINSTALL_HOOK="""
# run the test to install team system
cd ${WORKSPACE}/ariac-docker
bash -x ./prepare_ariac_system.bash ${ROS_DISTRO}
"""

INSTALL_JOB_POSTINSTALL_HOOK="""
"""
# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS=""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
