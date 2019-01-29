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

export BUILDING_SOFTWARE_DIRECTORY="ign-gazebo"
export BUILDING_JOB_REPOSITORIES="stable"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_GAZEBO_DEPENDENCIES"

export NEEDS_GZ11_SUPPORT=true
. "${SCRIPT_DIR}/lib/_gz11_hook.bash"

export BUILD_IGN_FUEL_TOOLS=true

if [[ $(date +%Y%m%d) -le 20190201 ]]; then
  # Enable prerelease and nightly repos until a certain date
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease nightly"
fi

export GPU_SUPPORT_NEEDED=true

. ${SCRIPT_DIR}/lib/generic-building-base.bash
