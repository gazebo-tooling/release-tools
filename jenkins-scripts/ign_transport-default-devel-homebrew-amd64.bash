#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify IGN_TRANSPORT_MAJOR_VERSION to help with dependency resolution
if grep 'ign_configure_project *(' \
        ${WORKSPACE}/ign-transport/CMakeLists.txt
then
  IGN_TRANSPORT_MAJOR_VERSION=$(\
    grep '^project.*VERSION' ${WORKSPACE}/ign-transport/CMakeLists.txt | \
    sed -e 's@.* VERSION \([0-9][0-9]*\).*@\1@')
else
  IGN_TRANSPORT_MAJOR_VERSION=$(\
    grep 'set.*IGN_TRANSPORT_MAJOR_VERSION ' ${WORKSPACE}/ign-transport/CMakeLists.txt | \
    tr -d 'a-zA-Z _()')
fi

# Drop version number if it is 1 (ignition-transport 1.X is in ignition-transport.rb)
if [ $IGN_TRANSPORT_MAJOR_VERSION -eq 1 ]; then
  IGN_TRANSPORT_MAJOR_VERSION=""
else
  rm -rf ${WORKSPACE}/ign-transport${IGN_TRANSPORT_MAJOR_VERSION}
  cp -R ${WORKSPACE}/ign-transport \
        ${WORKSPACE}/ign-transport${IGN_TRANSPORT_MAJOR_VERSION}
fi

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash \
  ignition-transport${IGN_TRANSPORT_MAJOR_VERSION}

