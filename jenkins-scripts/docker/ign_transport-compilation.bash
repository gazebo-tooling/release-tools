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

export BUILDING_SOFTWARE_DIRECTORY="ign-transport"
export BUILDING_PKG_DEPENDENCIES_VAR_NAME="IGN_TRANSPORT_DEPENDENCIES"
export BUILDING_JOB_REPOSITORIES="stable"
# Install Ignition Tools while we release a .deb package.
# ToDo: Remove this env variable after releasing Ignition Tools.
export DOCKER_POSTINSTALL_HOOK="""\
  apt-get update && \\
  apt-get install -y mercurial && \\
  hg clone https://bitbucket.org/ignitionrobotics/ign-tools &&  \\
  mkdir ign-tools/build && \\
  cd ign-tools/build &&  \\
  cmake .. && \\
  make install
"""

. ${SCRIPT_DIR}/lib/generic-building-base.bash
