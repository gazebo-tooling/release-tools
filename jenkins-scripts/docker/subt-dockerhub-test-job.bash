#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true
export USE_DOCKER_IN_DOCKER=true
# squid is blocking nvidia packages inside the dockerhub image
export USE_SQUID=false

. ${SCRIPT_DIR}/lib/_install_nvidia_docker.sh

export INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: testing by running dockerhub'

${INSTALL_NVIDIA_DOCKER2}

xhost +
timeout --preserve-status 300 ${DOCKER2_CMD} -v \"/etc/localtime:/etc/localtime:ro\" -v \"/dev/input:/dev/input\" --network host --privileged --security-opt seccomp=unconfined nkoenig/subt-virtual-testbed tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG1 || true

echo '# END SECTION'
"""

export DEPENDENCY_PKGS="${DEPENDENCY_PKGS} software-properties-common xauth x11-xserver-utils"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
