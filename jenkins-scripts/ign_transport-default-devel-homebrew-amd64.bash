#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

IS_A_HEAD_FORMULA=true

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash ignition-transport

