#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG=""
export INSTALL_JOB_REPOS=""

INSTALL_JOB_PREINSTALL_HOOK="""
# import the SRC repo
echo \"deb http://52.53.157.231/src ${DISTRO} main\" >\\
                                           /etc/apt/sources.list.d/src.list
apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
wget -qO - http://52.53.157.231/src/src.key | sudo apt-key add -
sudo apt-get update
"""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
