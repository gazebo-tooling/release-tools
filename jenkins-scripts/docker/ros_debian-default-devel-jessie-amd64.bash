#!/bin/bash -x

# Knowing Script dir beware of symlink. Trick for debian
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export LINUX_DISTRO=debian
export DISTRO=jessie

. ${SCRIPT_DIR}/lib/ros_debian-base-linux.bash
