#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export RELEASE_REPO_DIRECTORY=${DISTRO}
export ENABLE_ROS=false

# Use src temporary repo
export DOCKER_POSTINSTALL_HOOK="""\
echo \"deb http://srcsim.gazebosim.org/src ${DISTRO} main\" > /etc/apt/sources.list.d/src.list && \\
wget -qO - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add - && \\
apt-get update
"""

. ${SCRIPT_DIR}/lib/debbuild-base.bash
