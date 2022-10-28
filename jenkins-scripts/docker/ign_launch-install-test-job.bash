#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

[[ -z ${ENABLE_GZ_SIM_RUNTIME_TEST} ]] && ENABLE_GZ_SIM_RUNTIME_TEST=true

# Import library
. ${SCRIPT_DIR}/lib/_gazebo_utils.sh

if ${ENABLE_GZ_SIM_RUNTIME_TEST}; then
  export GPU_SUPPORT_NEEDED=true
  export INSTALL_JOB_POSTINSTALL_HOOK="""
GZ_SIM_RUNTIME_TEST_USE_IGN=${GZ_SIM_RUNTIME_TEST_USE_IGN:-false}
${GZ_SIM_RUNTIME_TEST}
"""
fi

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS="${DEPENDENCY_PKGS} wget bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
