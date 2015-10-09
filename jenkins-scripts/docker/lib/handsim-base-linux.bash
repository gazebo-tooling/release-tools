#!/bin/bash -x

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="handsim_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

SOFTWARE_DIR="handsim"
USE_OSRF_REPO=true
DEPENDENCY_PKGS="${HANDSIM_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/_generic_linux_compilation_build.sh.bash

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
