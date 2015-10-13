#!/bin/bash -x

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="robocup3ds_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

SOFTWARE_DIR="robocup3ds"
USE_OSRF_REPO=true
DEPENDENCY_PKGS="libgazebo6-dev libqt4-dev libboost-1.54-dev"

. ${SCRIPT_DIR}/lib/_generic_linux_compilation_build.sh.bash

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
