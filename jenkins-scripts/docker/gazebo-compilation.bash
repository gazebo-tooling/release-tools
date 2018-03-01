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
if [[ $(date +%Y%m%d) -le 20180201 ]]; then
   ## need prerelease repo to get ignition-cmake during the development cycle
   OSRF_REPOS_TO_USE="${OSRF_REPOS_TO_USE} prerelease"
fi

# Can not use generic compilation since we host the DART instalation and some
# other logic based of every gazebo version
. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
