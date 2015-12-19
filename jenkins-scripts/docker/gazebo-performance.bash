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

export GAZEBO_BASE_TESTS_HOOK="""
echo '# BEGIN SECTION: PERFORMANCE testing'
make test ARGS=\"-VV -R PERFORMANCE_*\" || true
echo '# END SECTION'
"""

# Can not use generic compilation since we host the DART instalation and some
# other logic based of every gazebo version
. ${SCRIPT_DIR}/lib/gazebo-base-default.bash
