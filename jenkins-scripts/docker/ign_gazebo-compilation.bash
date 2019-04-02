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

export BUILDING_SOFTWARE_DIRECTORY="ign-gazebo"
export BUILDING_JOB_REPOSITORIES="stable"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_GAZEBO_DEPENDENCIES"

# Enable prerelease repos until a certain date
if [[ $(date +%Y%m%d) -le 20190501 ]]; then
  export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease"
fi

# Identify IGN_GAZEBO_MAJOR_VERSION to help with dependency resolution
IGN_GAZEBO_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-gazebo/CMakeLists.txt)

# Check IGN_GAZEBO version is integer
if ! [[ ${IGN_GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_GAZEBO_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

# Build dependencies from source while we don't have nightlies for Blueprint
if [[ ${IGN_GAZEBO_MAJOR_VERSION} -ge 2 ]]; then
  export BUILD_IGN_RENDERING=true
  export IGN_RENDERING_BRANCH=default
  export BUILD_IGN_SENSORS=true
  export IGN_SENSORS_BRANCH=default
fi

export USE_GCC8=true

export GPU_SUPPORT_NEEDED=true

. ${SCRIPT_DIR}/lib/generic-building-base.bash
