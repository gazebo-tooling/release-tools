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

export BUILDING_SOFTWARE_DIRECTORY="${BUILDING_SOFTWARE_DIRECTORY:-ign-rendering}"

# Identify GZ_RENDERING_MAJOR_VERSION to help with dependency resolution
GZ_RENDERING_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/${BUILDING_SOFTWARE_DIRECTORY}/CMakeLists.txt)

# Check GZ_RENDERING version is integer
if ! [[ ${GZ_RENDERING_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GZ_RENDERING_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${GZ_RENDERING_MAJOR_VERSION} -ge 1 ]]; then
  export NEED_C17_COMPILER=true
fi

export GPU_SUPPORT_NEEDED=true
export GZDEV_PROJECT_NAME="gz-rendering${GZ_RENDERING_MAJOR_VERSION}"

# set SKIP_optix=true for Harmonic and earlier
# not needed for ionic since https://github.com/gazebosim/gz-rendering/pull/1032
if [[ ${GZ_RENDERING_MAJOR_VERSION} -le 8 ]]; then
  export BUILDING_EXTRA_CMAKE_PARAMS+=" -DSKIP_optix=true"
fi

. ${SCRIPT_DIR}/lib/generic-building-base.bash
