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

. ${SCRIPT_DIR}/lib/_sdformat_version_hook.bash

export ABI_JOB_SOFTWARE_NAME="sdformat"
export ABI_JOB_PKG_DEPENDENCIES_VAR_NAME="SDFORMAT_BASE_DEPENDENCIES"

if [[ ${SDFORMAT_MAJOR_VERSION} -ge 8 ]]; then
  export NEED_C17_COMPILER=true
fi

if [[ ${SDFORMAT_MAJOR_VERSION} -ge 10 ]]; then
  export ABI_JOB_HEADER_PREFIX=sdformat[0-9]*
fi

# default to use stable repos
export ABI_JOB_REPOS="stable"

# set GZDEV_PROJECT_NAME so it can override repos if necessary
export GZDEV_PROJECT_NAME=${ABI_JOB_SOFTWARE_NAME}${SDFORMAT_MAJOR_VERSION}

. ${SCRIPT_DIR}/lib/generic-abi-base.bash
