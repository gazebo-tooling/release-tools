#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

RERUN_FAILED_TESTS=1

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash ignition-math
