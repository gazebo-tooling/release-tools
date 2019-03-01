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

export BUILDING_SOFTWARE_DIRECTORY="ign-physics"
export BUILDING_JOB_REPOSITORIES="stable"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_PHYSICS_DEPENDENCIES"

export USE_GCC8=true

. ${SCRIPT_DIR}/lib/generic-building-base.bash
