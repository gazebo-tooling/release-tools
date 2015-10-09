#!/bin/bash -x
set -e

echo '# BEGIN SECTION: setup the testing enviroment'
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

SOFTWARE_DIR="ign_math"
USE_OSRF_REPO=false
DEPENDENCY_PKGS="${_BASE_DEPENDENCIES}"

source _generic_linux_compilation_build.sh.bash

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
