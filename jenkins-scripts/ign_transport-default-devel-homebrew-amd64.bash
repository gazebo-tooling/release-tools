#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify PROJECT_MAJOR_VERSION to help with dependency resolution
PROJECT_MAJOR_VERSION=`\
  grep 'set.*PROJECT_MAJOR_VERSION ' ${WORKSPACE}/ign-transport/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`
# Drop version number if it is 1 (ignition-transport 1.X is in ignition-transport.rb)
if [ $PROJECT_MAJOR_VERSION -eq 1 ]; then
  PROJECT_MAJOR_VERSION=""
else
  rm -rf ${WORKSPACE}/ign-transport${PROJECT_MAJOR_VERSION}
  cp -R ${WORKSPACE}/ign-transport \
        ${WORKSPACE}/ign-transport${PROJECT_MAJOR_VERSION}
fi

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash \
  ignition-transport${PROJECT_MAJOR_VERSION}

