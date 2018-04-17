#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Import library
. ${SCRIPT_DIR}/lib/_gazebo_utils.sh

INSTALL_JOB_POSTINSTALL_HOOK="""
${GAZEBO_RUNTIME_TEST}
"""
# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS="${DEPENDENCY_PKGS} wget bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
