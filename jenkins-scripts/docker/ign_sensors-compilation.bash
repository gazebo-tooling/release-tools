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

export BUILDING_SOFTWARE_DIRECTORY="ign-sensors"
# ign sensors compiles all its dependencies from gz11 branches
#export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_SENSORS_DEPENDENCIES"
export BUILDING_JOB_REPOSITORIES="stable"
if [[ $(date +%Y%m%d) -le 20180831 ]]; then
  ## need prerelease repo to get ignition-cmake1 for ign-rendering
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease"
fi

export BUILDING_DEPENDENCIES="lcov"

# TODO: stop building dependencies from source after there's a release
export IGN_CMAKE_BRANCH="gz11"
export IGN_MATH_BRANCH="gz11"
export IGN_SDFORMAT_BRANCH="gz11"
export IGN_COMMON_BRANCH="gz11"
export IGN_MSGS_BRANCH="gz11"
export IGN_TRANSPORT_BRANCH="gz11"
export IGN_RENDERING_BRANCH="gz11"

export BUILD_IGN_CMAKE=true
export BUILD_IGN_TOOLS=true
export BUILD_IGN_MATH=true
export BUILD_IGN_SDFORMAT=true
export BUILD_IGN_COMMON=true
export BUILD_IGN_MSGS=true
export BUILD_IGN_TRANSPORT=true
export BUILD_IGN_RENDERING=true

export GPU_SUPPORT_NEEDED=true

. ${SCRIPT_DIR}/lib/generic-building-base.bash
