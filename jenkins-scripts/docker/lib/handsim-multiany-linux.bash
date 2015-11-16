#!/bin/bash -x

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="handsim_multiany"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

cd $WORKSPACE

echo '# BEGIN SECTION: building ignition-transport: ${IGN_BRANCH}'
[[ -d ign-transport ]] && rm -fr ign-transport
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j${MAKE_JOBS}
make install
echo '# END SECTION'

echo '# BEGIN SECTION: building haptix-comm: ${IGN_BRANCH}'
[[ -d haptix-comm ]] && rm -fr haptix-comm
hg clone https://bitbucket.org/osrf/haptix-comm
cd haptix-comm
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j${MAKE_JOBS}
make install
echo '# END SECTION'

echo '# BEGIN SECTION: building handsim: ${HANDSIM_BRANCH}'
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/handsim \
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
cd $WORKSPACE/handsim
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
echo '# END SECTION'
DELIM

SOFTWARE_DIR="handsim"
USE_OSRF_REPO=true
# Dependencies only. No pkgs for: haptix or ignition
DEPENDENCY_PKGS="mercurial ca-certificates \\
                 ${BASE_DEPENDENCIES}  \\
                 ${IGN_TRANSPORT_DEPENDENCIES} \\
                 ${HAPTIX_COMM_DEPENDENCIES_WITHOUT_IGN} \\
		 ${HANDSIM_DEPENDENCIES_WITHOUT_HAPTIX}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
