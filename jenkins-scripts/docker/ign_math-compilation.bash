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

# Identify IGN_MATH_MAJOR_VERSION to help with dependency resolution
IGN_MATH_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-math/CMakeLists.txt)

# Check IGN_MATH version is integer
if ! [[ ${IGN_MATH_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_MATH_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

export BUILDING_JOB_REPOSITORIES="stable"
# gz11 hook will take of adding prerelease
. "${SCRIPT_DIR}/lib/_gz11_hook.bash"

export BUILDING_SOFTWARE_DIRECTORY="ign-math"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_MATH_DEPENDENCIES"

if ${NEEDS_GZ11_SUPPORT}; then
  export BUILD_IGN_CMAKE=true
fi

# To get ign-cmake1 package in prerelease
if [[ $(date +%Y%m%d) -le 20181231 ]]; then
  ## need prerelease repo to get ignition-cmake1 for ign-rendering
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease"
fi

. "${SCRIPT_DIR}/lib/generic-building-base.bash"
