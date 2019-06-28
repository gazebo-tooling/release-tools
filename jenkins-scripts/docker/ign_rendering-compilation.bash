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

export BUILDING_SOFTWARE_DIRECTORY="ign-rendering"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_RENDERING_DEPENDENCIES"

# Identify IGN_RENDERING_MAJOR_VERSION to help with dependency resolution
IGN_RENDERING_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-rendering/CMakeLists.txt)

# Check IGN_RENDERING version is integer
if ! [[ ${IGN_RENDERING_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_RENDERING_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${IGN_RENDERING_MAJOR_VERSION} -ge 1 ]]; then
  export USE_GCC8=true
fi

export GPU_SUPPORT_NEEDED=true
export GZDEV_PROJECT_NAME="ignition-rendering${IGN_RENDERING_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
