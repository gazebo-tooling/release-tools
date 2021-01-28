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
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_GAZEBO_DEPENDENCIES"

# Identify IGN_GAZEBO_MAJOR_VERSION to help with dependency resolution
IGN_GAZEBO_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-gazebo/CMakeLists.txt)

# Check IGN_GAZEBO version is integer
if ! [[ ${IGN_GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_GAZEBO_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

export NEED_C17_COMPILER=true
export GPU_SUPPORT_NEEDED=true

if [[ ${IGN_GAZEBO_MAJOR_VERSION} -eq 5 ]]; then
  export BUILD_IGN_RENDERING=true
  export IGN_RENDERING_MAJOR_VERSION=5
  export IGN_RENDERING_BRANCH=main

  export BUILD_IGN_GUI=true
  export IGN_GUI_MAJOR_VERSION=5
  export IGN_GUI_BRANCH=main

  export BUILD_IGN_SENSORS=true
  export IGN_SENSORS_MAJOR_VERSION=5
  export IGN_SENSORS_BRANCH=main

  export BUILD_IGN_MSGS=true
  export IGN_MSGS_MAJOR_VERSION=7
  export IGN_MSGS_BRANCH=main

  export BUILD_IGN_FUEL_TOOLS=true
  export IGN_FUEL_TOOLS_MAJOR_VERSION=6
  export IGN_FUEL_TOOLS_BRANCH=main

  export BUILD_IGN_TRANSPORT=true
  export IGN_TRANSPORT_MAJOR_VERSION=10
  export IGN_TRANSPORT_BRANCH=main

  export BUILD_SDFORMAT=true
  export SDFORMAT_MAJOR_VERSION=11
  export SDFORMAT_BRANCH=master

  export BUILD_IGN_PHYSICS=true
  export IGN_PHYSICS_MAJOR_VERSION=4
  export IGN_PHYSICS_BRANCH=main

  export BUILD_IGN_UTILS=true
  export IGN_UTILS_MAJOR_VERSION=1
  export IGN_UTILS_BRANCH=main
fi
export GZDEV_PROJECT_NAME="ignition-gazebo${IGN_GAZEBO_MAJOR_VERSION}"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
