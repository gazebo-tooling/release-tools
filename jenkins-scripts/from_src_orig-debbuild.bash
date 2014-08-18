#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export ENABLE_ROS=false
export UPLOAD_SOURCEDEB=true

. ${SCRIPT_DIR}/lib/debbuild-from-origsource.bash 
