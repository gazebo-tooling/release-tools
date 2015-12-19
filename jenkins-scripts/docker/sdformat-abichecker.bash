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

export ABI_JOB_SOFTWARE_NAME="sdformat"
export ABI_JOB_REPOS="stable"
export ABI_JOB_PKG_DEPENDENCIES_VAR_NAME="SDFORMAT_BASE_DEPENDENCIES"

. ${SCRIPT_DIR}/lib/generic-abi-base.bash
