#!/bin/bash -xe

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: cleanup brew installation'
. ${SCRIPT_DIR}/lib/_homebrew_cleanup.bash
echo '# END SECTION'

echo '# BEGIN SECTION: run the one-liner installation'
# TODO: change this to 'curl -ssL https://get.gazebosim.org' once that supports https
curl -ssL https://github.com/ignition-tooling/release-tools/raw/master/one-line-installations/gazebo.sh | sh -x
echo '# END SECTION'
