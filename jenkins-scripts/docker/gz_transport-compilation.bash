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

export BUILDING_SOFTWARE_DIRECTORY="${BUILDING_SOFTWARE_DIRECTORY:-ign-transport}"
  export BUILDING_PKG_DEPENDENCIES_VAR_NAME="GZ_TRANSPORT_DEPENDENCIES"

# Identify GZ_TRANSPORT_MAJOR_VERSION to help with dependency resolution
GZ_TRANSPORT_MAJOR_VERSION=$(\
  python3 ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/${BUILDING_SOFTWARE_DIRECTORY}/CMakeLists.txt)

# Check GZ_TRANSPORT version is integer
if ! [[ ${GZ_TRANSPORT_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GZ_TRANSPORT_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${GZ_TRANSPORT_MAJOR_VERSION} -ge 6 ]]; then
  export NEED_C17_COMPILER=true
fi

if [[ ${GZ_TRANSPORT_MAJOR_VERSION} -ge 15 ]]; then
  # gz-transport version >= 15 will use zenoh_cpp_vendor package from ROS 2
  # repo for zenoh support. Setting this env var will set up ROS env with the
  # specified ROS distro in the generated build.sh script.
  export ROS_DISTRO_SETUP_NEEDED="jazzy"
fi

export GZDEV_PROJECT_NAME="gz-transport${GZ_TRANSPORT_MAJOR_VERSION}"

. "${SCRIPT_DIR}/lib/generic-building-base.bash"
