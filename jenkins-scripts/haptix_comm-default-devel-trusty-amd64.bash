#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISTRO=trusty
export REPO_DIRECTORY="haptix_comm"
export PKG_DEPENDENCIES_VAR_NAME="HAPTIX_COMM_DEPENDENCIES"

. ${SCRIPT_DIR}/lib/generic-job.bash
