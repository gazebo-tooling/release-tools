#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

if [[ -z ${ARCH} ]]; then
  echo "ARCH variable not set!"
  exit 1
fi

if [[ -z ${DISTRO} ]]; then
  echo "DISTRO variable not set!"
  exit 1
fi

if [[ -z ${ROS_DISTRO} ]]; then
  echo "ROS_DISTRO variable not set!"
  exit 1
fi

. ${SCRIPT_DIR}/lib/debbuild-bloom-base.bash
