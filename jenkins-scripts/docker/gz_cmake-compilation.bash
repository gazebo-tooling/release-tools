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

export BUILDING_SOFTWARE_DIRECTORY="${BUILDING_SOFTWARE_DIRECTORY:-ign-cmake}"
export BUILDING_DEPENDENCIES="pkg-config"

# Enable long-running ign-cmake tests in CI.
export BUILDING_EXTRA_CMAKE_PARAMS+=" -DBUILDSYSTEM_TESTING=True"

# Identify GZ_CMAKE_MAJOR_VERSION to help with dependency resolution
GZ_CMAKE_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/${BUILDING_SOFTWARE_DIRECTORY}/CMakeLists.txt)

# Check GZ_CMAKE version is integer
if ! [[ ${GZ_CMAKE_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GZ_CMAKE_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

export GZDEV_PROJECT_NAME="gz-cmake${GZ_CMAKE_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
