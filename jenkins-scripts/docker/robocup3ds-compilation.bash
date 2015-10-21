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

export GPU_SUPPORT_NEEDED=true

export BUILDING_SOFTWARE_DIRECTORY="robocup3ds"
export BUILDING_DEPENDENCIES="libgazebo6-dev libqt4-dev libboost-dev"
export BUILDING_JOB_REPOSITORIES="stable prerelease"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
