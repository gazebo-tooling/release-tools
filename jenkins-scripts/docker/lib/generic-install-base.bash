#!/bin/bash -x
set -e

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

if [ `expr length "${INSTALL_JOB_PREINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre install hook'
${INSTALL_JOB_PREINSTALL_HOOK}
echo '# END SECTION'
fi

if [ `expr length "${INSTALL_JOB_PKG} "` -gt 1 ]; then
echo "# BEGIN SECTION: try to install package: ${INSTALL_JOB_PKG}"
apt-get install -y ${INSTALL_JOB_PKG}
echo '# END SECTION'
fi

if [ `expr length "${INSTALL_JOB_POSTINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running post install hook'
${INSTALL_JOB_POSTINSTALL_HOOK}
echo '# END SECTION'
fi
DELIM

OSRF_REPOS_TO_USE=${INSTALL_JOB_REPOS}

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
