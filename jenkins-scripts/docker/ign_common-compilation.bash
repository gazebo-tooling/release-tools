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

export BUILDING_SOFTWARE_DIRECTORY="ign-common"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="GZ_COMMON_DEPENDENCIES"

# Identify GZ_COMMON_MAJOR_VERSION to help with dependency resolution
GZ_COMMON_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-common/CMakeLists.txt)

# Check GZ_COMMON version is integer
if ! [[ ${GZ_COMMON_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GZ_COMMON_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${GZ_COMMON_MAJOR_VERSION} -ge 3 ]]; then
  export NEED_C17_COMPILER=true
fi

export GZDEV_PROJECT_NAME="gz-common${GZ_COMMON_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
