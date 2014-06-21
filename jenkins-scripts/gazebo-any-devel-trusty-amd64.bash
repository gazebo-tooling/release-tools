#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Hack to pick from current processes the DISPLAY available
export DISPLAY=$(ps aux | grep "X :" | grep -v grep | awk '{ print $12 }')

export DISTRO=trusty
export DART_FROM_PKGS=true
export DART_USE_4_VERSION=true

. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
