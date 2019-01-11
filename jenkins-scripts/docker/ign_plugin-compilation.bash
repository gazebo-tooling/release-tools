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
export BUILDING_JOB_REPOSITORIES="stable"
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
  export NEEDS_GZ11_SUPPORT=true
fi

. "${SCRIPT_DIR}/lib/_gz11_hook.bash"

# Enable prerelease and nightly repos until a certain date
if [[ $(date +%Y%m%d) -le 20190201 ]]; then
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease nightly"
fi

. ${SCRIPT_DIR}/lib/generic-building-base.bash
