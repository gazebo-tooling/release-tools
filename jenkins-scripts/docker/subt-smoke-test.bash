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

. ${SCRIPT_DIR}/lib/_subt_utils.sh

export ROS_SETUP_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: smoke test'
source ./install/setup.bash || true
${SUBT_COMPETITION_TEST}
echo 'Smoke testing completed successfully.'
echo '# END SECTION'
"""

. ${SCRIPT_DIR}/lib/subt-compilation-base.bash
