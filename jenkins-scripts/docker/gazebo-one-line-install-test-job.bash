#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG=""
export INSTALL_JOB_REPOS=""

. ${SCRIPT_DIR}/lib/_gazebo_utils.bash

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: run the one-liner installation'
curl -ssL http://get.gazebosim.org | sh
echo '# END SECTION'

${GAZEBO_RUNTIME_TEST}
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
