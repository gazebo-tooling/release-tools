#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

if [[ $DISTRO = 'precise' ]]; then
  export ROS_DISTRO=groovy
elif [[ $DISTRO = 'quantal' ]]; then
  export ROS_DISTRO=groovy
elif [[ $DISTRO = 'raring' ]]; then
  export ROS_DISTRO=hydro
elif [[ $DISTRO = 'saucy' ]]; then
  # saucy is not supported in drcsim yet
  exit 0
else
  echo "Unknow ubuntu distro. Fix your script"
  exit 1
fi

. ${SCRIPT_DIR}/lib/drcsim-check-release.bash
