#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify IGNITION_TRANSPORT_MAJOR_VERSION to help with dependency resolution
IGNITION_TRANSPORT_MAJOR_VERSION=`\
  grep 'set.*IGNITION_TRANSPORT_MAJOR_VERSION ' ${WORKSPACE}/ign-transport/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`
# Drop version number if it is 1 (ignition-transport 1.X is in ignition-transport.rb)
if [ $IGNITION_TRANSPORT_MAJOR_VERSION -eq 1 ]; then
  IGNITION_TRANSPORT_MAJOR_VERSION=""
else
  rm -rf ${WORKSPACE}/ign-transport${IGNITION_TRANSPORT_MAJOR_VERSION}
  cp -R ${WORKSPACE}/ign-transport \
        ${WORKSPACE}/ign-transport${IGNITION_TRANSPORT_MAJOR_VERSION}
fi

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash \
  ignition-transport${IGNITION_TRANSPORT_MAJOR_VERSION}

