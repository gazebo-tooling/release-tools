#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Do not check nightly
if [[ $VERSION = 'nightly' ]]; then
    echo "DO_NOT_CHECK"
    exit 0
fi

# Do not check i386
if [[ $ARCH = 'i386' ]]; then
    echo "DO_NOT_CHECK"
    exit 0
fi

# debbuild typically does not define ROS_DISTRO so autogenerate it
# bloom ros-gazebo-pkgs define ROS_DISTRO, ignore non supported drcsim 
if [[ $DISTRO = 'precise' ]]; then
  if [[ $ROS_DISTRO != 'groovy' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
  export ROS_DISTRO=groovy
elif [[ $DISTRO = 'quantal' ]]; then
  if [[ $ROS_DISTRO != 'groovy' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
  export ROS_DISTRO=groovy
elif [[ $DISTRO = 'raring' ]]; then
  if [[ $ROS_DISTRO != 'hydro' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
  export ROS_DISTRO=hydro
elif [[ $DISTRO = 'saucy' ]]; then
  # saucy is not supported in drcsim yet
  echo "DO_NOT_CHECK"
  exit 0
else
  echo "Unknow ubuntu distro. Fix your script"
  exit 1
fi

. ${SCRIPT_DIR}/lib/drcsim-check-release.bash
