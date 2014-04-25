#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISTRO=raring
export ROS_DISTRO=hydro
# No node with raring relieable GPU
export USE_DISPLAY=false

. ${SCRIPT_DIR}/lib/drcsim-base.bash
