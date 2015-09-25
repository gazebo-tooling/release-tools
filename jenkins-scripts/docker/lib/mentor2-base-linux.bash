#!/bin/bash -x

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="mentor2_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

echo '# BEGIN SECTION: configuring'
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/mentor2 \
    -DCMAKE_INSTALL_PREFIX=/usr
echo '# END SECTION'

echo '# BEGIN SECTION: compiling'
make -j${MAKE_JOBS}
echo '# END SECTION'

echo '# BEGIN SECTION: installing'
make install
echo '# END SECTION'

echo '# BEGIN SECTION: running tests'
mkdir -p \$HOME
make test ARGS="-VV" || true
echo '# END SECTION'

echo '# BEGIN SECTION: cppcheck'
cd $WORKSPACE/mentor2
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
echo '# END SECTION'
DELIM

SOFTWARE_DIR="mentor2"
OSRF_REPOS_TO_USE="mentor2"
DEPENDENCY_PKGS="${BASE_DEPENDENCIES} ${MENTOR2_DEPENDENCIES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
