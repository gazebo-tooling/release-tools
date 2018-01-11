#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
export SCRIPT_DIR="${SCRIPT_DIR%/*}"

if [[ -z ${ARCH} ]]; then
  echo "ARCH variable not set!"
  exit 1
fi

if [[ -z ${DISTRO} ]]; then
  echo "DISTRO variable not set!"
  exit 1
fi

export GAZEBO_EXPERIMENTAL_BUILD=true
export GAZEBO_BUILD_IGN_GUI=true
export GAZEBO_BUILD_IGN_COMMON=true
export GAZEBO_BUILD_SDFORMAT=true

. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
