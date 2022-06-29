#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

PIP_PACKAGES_NEEDED="psutil"

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash sdformat

