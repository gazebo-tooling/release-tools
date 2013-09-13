#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export DISPLAY=$(ps aux | grep "X :" | grep -v grep | awk '{ print $12 }')
export DISTRO=quantal
export COVERAGE_ENABLED=true
. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
