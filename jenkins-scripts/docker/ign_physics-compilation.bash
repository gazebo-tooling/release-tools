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

export BUILDING_JOB_REPOSITORIES="stable"
# gz11 hook will take of adding prerelease
. "${SCRIPT_DIR}/lib/_gz11_hook.bash"

export BUILDING_SOFTWARE_DIRECTORY="ign-physics"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_PHYSICS_DEPENDENCIES"
export DART_FROM_PKGS="true"

if ${NEEDS_GZ11_SUPPORT}; then
  export BUILD_IGN_CMAKE=true
  export BUILD_IGN_MATH=true
  export BUILD_IGN_PLUGINS=true
  export BUILD_SDFORMAT=true
fi

if [[ $(date +%Y%m%d) -le 20180831 ]]; then
  ## need prerelease repo to get ignition-cmake during the development cycle
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease"
fi

. ${SCRIPT_DIR}/lib/generic-building-base.bash
