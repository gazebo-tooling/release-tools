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

# Only precise and raring are supported
if [[ $DISTRO != 'precise' ]] && [[ $DISTRO != 'raring' ]]; then
  echo "DO_NOT_CHECK"
  exit 0
fi

# If no ROS_DISTRO is set, we stop processing here and succesffully end
if [[ -z $ROS_DISTRO ]]; then
    exit 0
fi

# Indigo is not supported by drcsim
if [[ $ROS_DISTRO = 'indigo' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
fi

# Exclude of checking versions not supported by drcsim
if [[ $DISTRO = 'raring' ]]; then
  if [[ $ROS_DISTRO != 'hydro' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
fi
