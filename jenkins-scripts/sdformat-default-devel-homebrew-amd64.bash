#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify SDFORMAT_MAJOR_VERSION to help with dependency resolution
SDFORMAT_MAJOR_VERSION=`\
  grep 'set.*SDF_MAJOR_VERSION ' ${WORKSPACE}/sdformat/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`
# Drop version number if it is less than 3 (sdformat 1.4, 2.2 in sdformat.rb)
if [ $SDFORMAT_MAJOR_VERSION -le 2 ]; then
  SDFORMAT_MAJOR_VERSION=""
else
  rm -f ${WORKSPACE}/sdformat${SDFORMAT_MAJOR_VERSION}
  ln -s ${WORKSPACE}/sdformat ${WORKSPACE}/sdformat${SDFORMAT_MAJOR_VERSION}
fi

PIP_PACKAGES_NEEDED="psutil"

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash sdformat${SDFORMAT_MAJOR_VERSION}

