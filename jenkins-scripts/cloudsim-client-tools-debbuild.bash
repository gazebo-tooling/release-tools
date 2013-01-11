#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Note:
# It is using the debbuild base. From there, importing ros and drcsim
# repositories are really not needed, but prefer to share code.
# Leave this file different in prevission of future more different
# dependencies

. ${SCRIPT_DIR}/lib/debbuild-base.bash

