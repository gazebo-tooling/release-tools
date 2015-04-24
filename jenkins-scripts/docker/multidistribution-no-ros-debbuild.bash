#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export ENABLE_ROS=false
export WORKAROUND_PBUILDER_BUG=true

. ${SCRIPT_DIR}/lib/debbuild-base.bash
