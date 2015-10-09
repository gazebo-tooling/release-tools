#!/bin/bash -x
set -e

# Identify SDFORMAT_MAJOR_VERSION to help with dependency resolution
SDFORMAT_MAJOR_VERSION=`\
  grep 'set.*SDF_MAJOR_VERSION ' ${WORKSPACE}/sdformat/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check sdformat version is integer
if ! [[ ${SDFORMAT_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
   echo "Error! SDFORMAT_MAJOR_VERSION is not an integer, check the detection"
   exit -1
fi

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="sdformat_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

SOFTWARE_DIR="sdformat"
source _generic_linux_compilation_build.sh.bash

USE_OSRF_REPO=true
DEPENDENCY_PKGS="${SDFORMAT_BASE_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
