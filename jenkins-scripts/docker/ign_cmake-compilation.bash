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

export BUILDING_SOFTWARE_DIRECTORY="ign-cmake"
export BUILDING_DEPENDENCIES="pkg-config"

# Identify IGN_CMAKE_MAJOR_VERSION to help with dependency resolution
IGN_CMAKE_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-cmake/CMakeLists.txt)

# Check IGN_CMAKE version is integer
if ! [[ ${IGN_CMAKE_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_CMAKE_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

export GZDEV_PROJECT_NAME="ignition-cmake${IGN_CMAKE_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
