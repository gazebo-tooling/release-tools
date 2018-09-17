#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export RELEASE_REPO_DIRECTORY=${DISTRO}
export ENABLE_ROS=true
export WORKAROUND_PBUILDER_BUG=true

. ${SCRIPT_DIR}/lib/_sdformat_version_hook.bash
. ${SCRIPT_DIR}/lib/debbuild-base.bash

