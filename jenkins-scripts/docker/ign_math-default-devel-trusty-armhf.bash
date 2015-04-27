#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export ARCH=armhf
export DISTRO=trusty

. ${SCRIPT_DIR}/lib/ign_math-base-linux.bash
