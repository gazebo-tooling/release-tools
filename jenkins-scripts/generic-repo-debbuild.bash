#!/bin/bash -x

# This script is to use with a git repo which contains the the source and the
# debian directory already in place, but it is *not* using git-buildpackage 
# format. Plain git + debian.

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/generic-repo-release.bash
