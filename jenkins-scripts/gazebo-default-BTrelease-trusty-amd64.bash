#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISTRO=trusty

export GZ_BUILD_TYPE=Release

. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
