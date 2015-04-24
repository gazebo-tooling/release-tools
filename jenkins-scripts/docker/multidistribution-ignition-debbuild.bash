#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export RELEASE_REPO_DIRECTORY=${DISTRO}
export ENABLE_ROS=false
export BITBUCKET_REPO=ignitionrobotics
export NEED_C11_COMPILER=true

. ${SCRIPT_DIR}/lib/debbuild-base.bash
