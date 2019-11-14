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

export BUILDING_SOFTWARE_DIRECTORY="ign-launch"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_LAUNCH_DEPENDENCIES"
export BUILD_IGN_TOOLS=true
export IGN_TOOLS_BRANCH=launch_part2

# Identify IGN_LAUNCH_MAJOR_VERSION to help with dependency resolution
IGN_LAUNCH_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-launch/CMakeLists.txt)

# Check IGN_LAUNCH version is integer
if ! [[ ${IGN_LAUNCH_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_LAUNCH_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${IGN_LAUNCH_MAJOR_VERSION} -eq 2 ]]; then
  export BUILD_IGN_MSGS=true
  export IGN_MSGS_MAJOR_VERSION=5
  export IGN_MSGS_BRANCH=default
  export BUILD_IGN_TRANSPORT=true
  export IGN_TRANSPORT_MAJOR_VERSION=8
  export IGN_TRANSPORT_BRANCH=default
  export BUILD_IGN_RENDERING=true
  export IGN_RENDERING_MAJOR_VERSION=3
  export IGN_RENDERING_BRANCH=default
  export BUILD_IGN_SENSORS=true
  export IGN_SENSORS_MAJOR_VERSION=3
  export IGN_SENSORS_BRANCH=default
  export BUILD_IGN_GUI=true
  export IGN_GUI_MAJOR_VERSION=3
  export IGN_GUI_BRANCH=default
  export BUILD_IGN_GAZEBO=true
  export IGN_GAZEBO_MAJOR_VERSION=3
  export IGN_GAZEBO_BRANCH=default
fi

export USE_GCC8=true

export GZDEV_PROJECT_NAME="ignition-launch${IGN_LAUNCH_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
