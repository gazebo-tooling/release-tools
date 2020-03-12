#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L "${0}" ]] && SCRIPT_DIR=$(readlink "${0}") || SCRIPT_DIR="${0}"
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export OSRF_REPOS_TO_USE=""

. "${SCRIPT_DIR}/lib/debian-git-repo-base.bash"
