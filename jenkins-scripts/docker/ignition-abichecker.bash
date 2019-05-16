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

# In ignition this variable is passed directly from the DSL job creation
if [[ -z ${ABI_JOB_SOFTWARE_NAME} ]]; then
  echo "ABI_JOB_SOFTWARE_NAME not set!"
  exit 1
fi

# convert from ign-package to IGN_PACKAGE_DEPENDENCIES
IGN_NAME_PREFIX=$(\
  echo ${ABI_JOB_SOFTWARE_NAME} | tr '[:lower:]-' '[:upper:]_')
ABI_JOB_PKG_DEPENDENCIES_VAR_NAME=${IGN_NAME_PREFIX}_DEPENDENCIES
DART_FROM_PKGS_VAR_NAME=${IGN_NAME_PREFIX}_DART_FROM_PKGS

# Identify IGN_MSGS_MAJOR_VERSION to help with dependency resolution
export ${IGN_NAME_PREFIX}_MAJOR_VERSION=$(\
  python ${SCRIPT_DIR}/../tools/detect_cmake_major_version.py \
  ${WORKSPACE}/${ABI_JOB_SOFTWARE_NAME}/CMakeLists.txt)

export ABI_JOB_REPOS="stable"

# Enable prerelease repos until a certain date
if [[ "${ABI_JOB_SOFTWARE_NAME}" = "ign-gazebo" ]]; then
  if [[ $(date +%Y%m%d) -le 20190521 ]]; then
    export ABI_JOB_REPOS="${ABI_JOB_REPOS} prerelease"
  fi
fi

. ${SCRIPT_DIR}/lib/generic-abi-base.bash
