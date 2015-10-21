echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="building_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

# Could be empty, just fine
if [[ "${BUILDING_PKG_DEPENDENCIES_VAR_NAME}" != "" ]]; then
  eval ARCHIVE_PROJECT_DEPENDECIES=\$${BUILDING_PKG_DEPENDENCIES_VAR_NAME}
fi

OSRF_REPOS_TO_USE=${BUILDING_JOB_REPOSITORIES}
DEPENDENCY_PKGS="${BASE_DEPENDENCIES} ${ARCHIVE_PROJECT_DEPENDECIES} ${BUILDING_DEPENDENCIES}"
SOFTWARE_DIR="${BUILDING_SOFTWARE_DIRECTORY}"

. ${SCRIPT_DIR}/lib/_generic_linux_compilation_build.sh.bash

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
