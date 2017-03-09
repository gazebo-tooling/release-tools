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

export BUILDING_SOFTWARE_DIRECTORY="ign-msgs"
export BUILDING_DEPENDENCIES="libprotobuf-dev libprotoc-dev protobuf-compiler ruby ruby-dev libignition-math3-dev"
export DOCKER_POSTINSTALL_HOOK="gem install protobuf"
export BUILDING_JOB_REPOSITORIES="stable"

. ${SCRIPT_DIR}/lib/generic-building-base.bash
