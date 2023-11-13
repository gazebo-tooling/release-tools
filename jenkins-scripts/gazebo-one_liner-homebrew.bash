#!/bin/bash -xe

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: cleanup brew installation'
. ${SCRIPT_DIR}/lib/_homebrew_cleanup.bash
echo '# END SECTION'

echo '# BEGIN SECTION: run the one-liner installation'
curl -sSL https://get.gazebosim.org | sh -x
# to test undeployed changes to gazebo.sh, comment out the
# curl invocation and uncomment the line below
# sh -x ${SCRIPT_DIR}/../one-line-installations/gazebo.sh
echo '# END SECTION'
