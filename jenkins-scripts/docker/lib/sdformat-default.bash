#!/bin/bash -x
set -e
cp ${SCRIPT_DIR}/../lib/_time_lib.sh ${WORKSPACE} && source ${WORKSPACE}/_time_lib.sh ${WORKSPACE}

init_stopwatch TOTAL_TIME
init_stopwatch CREATE_TESTING_ENVIROMENT

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

cat > build.sh << DELIM
#!/bin/bash
###################################################
# Make project-specific changes here
#
source ${WORKSPACE}/_time_lib.sh ${WORKSPACE}

set -ex

echo '# BEGIN SECTION: configure sdformat ${SDFORMAT_MAJOR_VERSION}'
# Step 2: configure and build
cd $WORKSPACE
cd $WORKSPACE/build
cmake $WORKSPACE/sdformat
echo '# END SECTION'

echo '# BEGIN SECTION: compiling'
init_stopwatch COMPILATION
make -j${MAKE_JOBS}
echo '# END SECTION'

echo '# BEGIN SECTION: installing'
make install
stop_stopwatch COMPILATION
echo '# END SECTION'

echo '# BEGIN SECTION: running tests'
init_stopwatch TEST
mkdir -p \$HOME
make test ARGS="-VV" || true
stop_stopwatch TEST
echo '# END SECTION'

echo '# BEGIN SECTION: cppcheck'
cd $WORKSPACE/sdformat
init_stopwatch CPPCHECK
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
stop_stopwatch CPPCHECK
echo '# END SECTION'
DELIM

USE_OSRF_REPO=true
DEPENDENCY_PKGS="${SDFORMAT_BASE_DEPENDENCIES}"
SOFTWARE_DIR="sdformat"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
