echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="drake_standalone"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
#
set -ex

echo '# Download drake-release repo'
[[ -d ${WORKSPACE}/drake-release ]] && rm -fr drake-release
git clone https://github.com/j-rivero/drake-release ${WORKSPACE}/drake-release
echo '# END SECTION'

echo '# BEGIN SECTION: run setup.bash'
cd ${WORKSPACE}/drake-release
./setup.bash
echo '# END SECTION'

echo '# BEGIN SECTION: run setup.bash'
cd ${WORKSPACE}/drake-release
./release-new-snapshot.bash
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE="stable drake"
DEPENDENCY_PKGS="sudo \
		 bash \
		 git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
