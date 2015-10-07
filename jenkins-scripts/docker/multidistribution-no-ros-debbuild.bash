#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export ENABLE_ROS=false

# Need to find a better place to handle project repository constraints
if [[ -n ${UPLOAD_TO_REPO} ]] && [[ ${UPLOAD_TO_REPO} == "mentor2" ]]; then
  OSRF_REPOS_TO_USE="mentor2"
fi

export OSRF_REPOS_TO_USE=${OSRF_REPOS_TO_USE:=stable}

. ${SCRIPT_DIR}/lib/debbuild-base.bash
