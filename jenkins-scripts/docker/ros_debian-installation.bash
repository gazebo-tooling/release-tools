#!/bin/bash -x

# Knowing Script dir beware of symlink. Trick for debian
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

if [[ -z ${LINUX_DISTRO} ]]; then
  echo "LINUX_DISTRO variable not set!"
  exit 1
fi

. ${SCRIPT_DIR}/lib/ros_debian-base-linux.bash
