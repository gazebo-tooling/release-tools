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

# Identify IGN_COMMON_MAJOR_VERSION to help with dependency resolution
IGN_COMMON_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-common/CMakeLists.txt)

# Check IGN_COMMON version is integer
if ! [[ ${IGN_COMMON_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_COMMON_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

export BUILDING_SOFTWARE_DIRECTORY="ign-common"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_COMMON_DEPENDENCIES"
export BUILDING_JOB_REPOSITORIES="stable"
if [[ $(date +%Y%m%d) -le 20180831 ]]; then
  ## need prerelease repo to get ignition-cmake during the development cycle
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease"
fi

. ${SCRIPT_DIR}/lib/generic-building-base.bash
