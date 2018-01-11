#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

. ${SCRIPT_DIR}/lib/_drake_lib.bash

DEBIAN_GIT_PREINSTALL_HOOK="""\
${DRAKE_BAZEL_INSTALL}
"""

. ${SCRIPT_DIR}/lib/debian-git-repo-base.bash
