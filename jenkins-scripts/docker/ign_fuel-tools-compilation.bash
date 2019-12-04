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

export BUILDING_SOFTWARE_DIRECTORY="ign-fuel-tools"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_FUEL_TOOLS_DEPENDENCIES"
export BUILDING_JOB_REPOSITORIES="stable"

# Identify IGN_FUEL_TOOLS_MAJOR_VERSION to help with dependency resolution
IGN_FUEL_TOOLS_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-fuel-tools/CMakeLists.txt)

# Check IGN_FUEL_TOOLS version is integer
if ! [[ ${IGN_FUEL_TOOLS_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_FUEL_TOOLS_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${IGN_FUEL_TOOLS_MAJOR_VERSION} -eq 4 ]]; then
  export BUILD_IGN_MSGS=true
  export IGN_MSGS_MAJOR_VERSION=5
  export IGN_MSGS_BRANCH=default
fi

. ${SCRIPT_DIR}/lib/generic-building-base.bash
