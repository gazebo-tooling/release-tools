#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
GAZEBO_MAJOR_VERSION=`\
  grep 'set.*GAZEBO_MAJOR_VERSION ' ${WORKSPACE}/gazebo/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`
# Drop version number if it is 1 (gazebo 1.9 is in gazebo.rb)
if [ $GAZEBO_MAJOR_VERSION -eq 1 ]; then
  GAZEBO_MAJOR_VERSION=""
else
  ln -s ${WORKSPACE}/gazebo ${WORKSPACE}/gazebo${GAZEBO_MAJOR_VERSION}
fi

. ${SCRIPT_DIR}/lib/project-default-devel-homebrew-amd64.bash gazebo${GAZEBO_MAJOR_VERSION} \
  "--with-ffmpeg --with-bullet --with-simbody"

