#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Need for priviledge mode (docker in docker)
export GPU_SUPPORT_NEEDED=true

if [[ -z ${ROS_DISTRO} ]]; then
    echo "Need to define a ROS_DISTRO value"
    exit
fi

INSTALL_JOB_PREINSTALL_HOOK="""
# run the test to install team system
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
apt-get update
apt-get install -y docker-ce

cd ${WORKSPACE}/ariac-docker
bash -x ./prepare_ariac_system.bash ${ROS_DISTRO}
"""

INSTALL_JOB_POSTINSTALL_HOOK="""
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
