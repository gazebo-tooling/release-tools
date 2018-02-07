echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="drake_standalone"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
#
set -ex

echo '# Download drake-release repo'
[[ -d ${WORKSPACE}/drake-release-tools ]] && rm -fr ${WORKSPACE}/drake-release-tools
git clone https://github.com/j-rivero/drake-release-tools ${WORKSPACE}/drake-release-tools
echo '# END SECTION'

echo '# BEGIN SECTION: run release new snapshot'
cd ${WORKSPACE}/drake-release-tools
/bin/bash -e builder/release-new-snapshot.bash
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE="drake"
DEPENDENCY_PKGS="sudo \
		 bash \
		 git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
