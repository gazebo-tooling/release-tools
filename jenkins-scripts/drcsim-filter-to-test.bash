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

# Exclude of checking versions not supported by drcsim
if [[ $DISTRO = 'precise' ]]; then
    # TODO: we are skipping precise-hydro by the moment
    # TODO: supporting it needs changes in drcsin-release-test.bash
    if [[ $ROS_DISTRO != 'groovy' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
elif [[ $DISTRO = 'quantal' ]]; then
  if [[ $ROS_DISTRO != 'groovy' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
elif [[ $DISTRO = 'raring' ]]; then
  if [[ $ROS_DISTRO != 'hydro' ]]; then
      echo "DO_NOT_CHECK"
      exit 0
  fi
elif [[ $DISTRO = 'saucy' ]]; then
  # saucy is not supported in drcsim
  echo "DO_NOT_CHECK"
  exit 0
else
  echo "Unknow ubuntu distro. Fix your script"
  exit 1
fi
