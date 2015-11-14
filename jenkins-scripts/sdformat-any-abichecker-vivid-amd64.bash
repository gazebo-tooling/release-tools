#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

if [[ -z ${DISTRO} ]]; then
  export DISTRO=vivid
fi

export REPO_TO_USE=OSRF

. ${SCRIPT_DIR}/lib/sdformat-any-base.bash
