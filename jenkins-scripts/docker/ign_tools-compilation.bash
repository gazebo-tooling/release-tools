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

export BUILDING_SOFTWARE_DIRECTORY="${BUILDING_SOFTWARE_DIRECTORY:-ign-tools}"
export BUILDING_JOB_REPOSITORIES="stable"
export BUILDING_DEPENDENCIES="ruby"

GZ_TOOLS_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-tools/CMakeLists.txt)

# Check GZ_TOOLS version is integer
if ! [[ ${GZ_TOOLS_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GZ_TOOLS_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

export GZDEV_PROJECT_NAME="gz-tools${GZ_TOOLS_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
