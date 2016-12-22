#!/bin/bash -x

# Parameters
#  - GAZEBO_BASE_CMAKE_ARGS (optional) extra arguments to pass to cmake
#  - GAZEBO_BASE_TESTS_HOOK (optional) [default to run UNIT, INTEGRATION, REGRESSION, EXAMPLE]
#                           piece of code to run in the testing section
#  - GAZEBO_BUILD_$DEP      (optional) [default false] 
#                           build dependencies from source. 
#                           DEP = SDFORMAT | IGN_MATH | IGN_TRANSPORT
#                           branch parameter = $DEP_BRANCH

#stop on error
set -e

GAZEBO_OSRF_DEPS="SDFORMAT IGN_MATH IGN_TRANSPORT"

. ${SCRIPT_DIR}/lib/_gazebo_version_hook.bash

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="gazebo_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# If Coverage build type was supplied in GAZEBO_BASE_CMAKE_ARGS, add lcov
# package.
if [[ ${GAZEBO_BASE_CMAKE_ARGS} != ${GAZEBO_BASE_CMAKE_ARGS/Coverage} ]]; then
  EXTRA_PACKAGES="${EXTRA_PACKAGES} lcov" 
fi

if [[ $GAZEBO_MAJOR_VERSION -lt 8 ]]; then
  GAZEBO_BASE_CMAKE_ARGS="${GAZEBO_BASE_CMAKE_ARGS} -DENABLE_TESTS_COMPILATION=True"
fi

cat > build.sh << DELIM_DART
###################################################
# Make project-specific changes here
#
set -ex
source ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}

# Step 2: configure and build
# Check for DART
if $DART_COMPILE_FROM_SOURCE; then
  echo '# BEGIN SECTION: compiling DART from source'
  if [ -d $WORKSPACE/dart ]; then
      cd $WORKSPACE/dart
      git pull
  else
     git clone https://github.com/dartsim/dart.git $WORKSPACE/dart
  fi
  rm -fr $WORKSPACE/dart/build
  mkdir -p $WORKSPACE/dart/build
  cd $WORKSPACE/dart/build
  cmake .. \
      -DCMAKE_INSTALL_PREFIX=/usr
  #make -j${MAKE_JOBS}
  make -j1
  make install
  echo '# END SECTION'
fi
DELIM_DART

# Process the source build of dependencies if needed
for dep_uppercase in $GAZEBO_OSRF_DEPS; do
  dep=`echo $dep_uppercase | tr '[:upper:]' '[:lower:]'`
  EXTRA_PACKAGES="${EXTRA_PACKAGES} mercurial"
  eval dependecy_installation="\$GAZEBO_BUILD_$dep_uppercase"

  if [[ -n ${dependecy_installation} ]] && ${dependecy_installation}; then
      # Handle the depedency BRANCH
      eval dep_branch=\$$dep_uppercase\_BRANCH
      [[ -z ${dep_branch} ]] && dep_branch='default'
cat >> build.sh << DELIM_BUILD_DEPS  
    echo "# BEGIN SECTION: building dependency: ${dep} (${dep_branch})"
    echo '# END SECTION'
    rm -fr $WORKSPACE/$dep

    if [[ ${dep/ign} == ${dep} ]]; then
      bitbucket_repo="osrf/${dep}"
    else
      # need to replace _ by -
      bitbucket_repo="ignitionrobotics/${dep/_/-}"
    fi

    hg clone http://bitbucket.org/\$bitbucket_repo -b ${dep_branch} \
	$WORKSPACE/$dep 

    GENERIC_ENABLE_CPPCHECK=false
    GENERIC_ENABLE_TESTS=false 
    SOFTWARE_DIR=$dep
    cd $WORKSPACE
    . ${SCRIPT_DIR}/lib/_generic_linux_compilation.bash
    cd $WORKSPACE &&  rm -fr $WORKSPACE/build
DELIM_BUILD_DEPS
  fi
done

cat >> build.sh << DELIM
# Normal cmake routine for Gazebo
echo '# BEGIN SECTION: Gazebo configuration'
rm -rf $WORKSPACE/install
mkdir -p $WORKSPACE/install $WORKSPACE/build
cd $WORKSPACE/build
cmake ${GAZEBO_BASE_CMAKE_ARGS}      \\
    -DCMAKE_INSTALL_PREFIX=/usr      \\
    -DENABLE_SCREEN_TESTS:BOOL=False \\
  $WORKSPACE/gazebo
echo '# END SECTION'

echo '# BEGIN SECTION: Gazebo compilation'
init_stopwatch COMPILATION
make -j${MAKE_JOBS}
stop_stopwatch COMPILATION
echo '# END SECTION'

if [[ $GAZEBO_MAJOR_VERSION -ge 8 ]]; then
  echo '# BEGIN SECTION: Tests compilation'
  init_stopwatch TESTS_COMPILATION
  make -j${MAKE_JOBS} tests
  stop_stopwatch TESTS_COMPILATION
  echo '# END SECTION'
fi

echo '# BEGIN SECTION: Gazebo installation'
make install
. /usr/share/gazebo/setup.sh
echo '# END SECTION'

# Need to clean up from previous built
rm -fr $WORKSPACE/cppcheck_results
rm -fr $WORKSPACE/test_results

# Run tests
if [ `expr length "${GAZEBO_BASE_TESTS_HOOK} "` -gt 1 ]; then
  ${GAZEBO_BASE_TESTS_HOOK}
  : # keep this line, needed if the variable is empty
else
  # Run default
  init_stopwatch TEST
  echo '# BEGIN SECTION: UNIT testing'
  make test ARGS="-VV -R UNIT_*" || true
  echo '# END SECTION'
  echo '# BEGIN SECTION: INTEGRATION testing'
  make test ARGS="-VV -R INTEGRATION_*" || true
  echo '# END SECTION'
  echo '# BEGIN SECTION: REGRESSION testing'
  make test ARGS="-VV -R REGRESSION_*" || true
  echo '# END SECTION'
  echo '# BEGIN SECTION: EXAMPLE testing'
  make test ARGS="-VV -R EXAMPLE_*" || true
  stop_stopwatch TEST
  echo '# END SECTION'
fi

echo '# BEGIN SECTION: running cppcheck'
init_stopwatch CPPCHECK
# Step 3: code check
cd $WORKSPACE/gazebo
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
stop_stopwatch CPPCHECK
echo '# END SECTION'
DELIM

USE_OSRF_REPO=true
SOFTWARE_DIR="gazebo"
DEPENDENCY_PKGS="${BASE_DEPENDENCIES} \
                 ${GAZEBO_BASE_DEPENDENCIES} \
		 ${GAZEBO_EXTRA_DEPENDENCIES} \
		 ${EXTRA_PACKAGES}"

# Need for cmake DISPLAY check (it uses xwininfo command)
if [[ $USE_GPU_DOCKER ]]; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} x11-utils"
fi

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
