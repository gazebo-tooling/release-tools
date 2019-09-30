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

export DART_FROM_PKGS=true

OSRF_REPOS_TO_USE="stable"

. ${SCRIPT_DIR}/lib/_gazebo_version_hook.bash

if [[ $GAZEBO_MAJOR_VERSION -ge 11 ]]; then
  USE_GCC8=1
fi

# Can not use generic compilation since we host the DART instalation and some
# other logic based of every gazebo version
. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
