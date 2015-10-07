#!/bin/bash -x
set -e

if [[ -z ${INSTALL_JOB_PKG} ]]; then
  echo "PKG_TO_INSTALL empty! needed to run the job"
  exit 1
fi

if [[ -z ${INSTALL_JOB_REPOS} ]]; then
  echo "REPOS_TO_INSTALL is empty. Default to stable"
  REPOS_TO_INSTALL="stable"
fi

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="install_job"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
#!/bin/bash

###################################################
# Make project-specific changes here
#
set -ex

if [ ${INSTALL_JOB_PREINSTALL_HOOK} ]; then
echo '# BEGIN SECTION: running pre install hook'
${INSTALL_JOB_PREINSTALL_HOOK}
echo '# END SECTION'
fi

echo '# BEGIN SECTION: try to install package: ${INSTALL_JOB_PKG}'
apt-get install -y ${INSTALL_JOB_PKG}
echo '# END SECTION'

if [ -n ${INSTALL_JOB_POSTINSTALL_HOOK} ]; then
echo '# BEGIN SECTION: running post install hook'
${INSTALL_JOB_POSTINSTALL_HOOK}
echo '# END SECTION'
fi
DELIM

OSRF_REPOS_TO_USE=${INSTALL_JOB_REPOS}

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
