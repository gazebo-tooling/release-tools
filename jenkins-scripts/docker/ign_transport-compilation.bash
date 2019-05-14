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

export BUILDING_SOFTWARE_DIRECTORY="ign-transport"
export BUILDING_JOB_REPOSITORIES="stable"
  export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_TRANSPORT_DEPENDENCIES"

# Identify IGN_TRANSPORT_MAJOR_VERSION to help with dependency resolution
IGN_TRANSPORT_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/ign-transport/CMakeLists.txt)

# Check IGN_TRANSPORT version is integer
if ! [[ ${IGN_TRANSPORT_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! IGN_TRANSPORT_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

if [[ ${IGN_TRANSPORT_MAJOR_VERSION} -ge 6 ]]; then
  export USE_GCC8=true
fi

if [[ ${IGN_TRANSPORT_MAJOR_VERSION} -ge 7 ]]; then
  if [[ $(date +%Y%m%d) -le 20190521 ]]; then
    export BUILDING_JOB_REPOSITORIES="${BUILDING_JOB_REPOSITORIES} prerelease"
  fi
fi

. "${SCRIPT_DIR}/lib/generic-building-base.bash"
