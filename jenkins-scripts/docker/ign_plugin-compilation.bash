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

export BUILDING_SOFTWARE_DIRECTORY="ign-plugin"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_PLUGIN_DEPENDENCIES"

# Identify IGN_PLUGIN_MAJOR_VERSION to help with dependency resolution
IGN_PLUGIN_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-plugin/CMakeLists.txt)

# Check IGN_PLUGIN version is integer
if ! [[ ${IGN_PLUGIN_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_PLUGIN_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${IGN_PLUGIN_MAJOR_VERSION} -ge 6 ]]; then
  export USE_GCC8=true
fi

export GZDEV_PROJECT_NAME="ignition-plugin${IGN_PLUGIN_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
