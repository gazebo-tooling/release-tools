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

cat > build.sh << DELIM_DART
###################################################
# Make project-specific changes here
#
set -ex

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

  if $(eval $\GAZEBO_BUILD_$dep_uppercase); then
      # Handle the depedency BRANCH
      $(eval dep_branch=$\GAZEBO_BUILD_$dep_uppercase\_BRANCH)
      [[ -z ${dep_branch} ]] && dep_branch='default'
cat >> build.sh << DELIM_BUILD_DEPS  
    echo "#BEGIN SECTION: building dependency: ${dep} (${dep_branch})"
    if [[ ${dep/IGN} == ${dep} ]]; then
      bitbucket_repo="osrf/${dep}"
    else
      bitbucket_repo="ignitionrobotics/${dep}"
    fi

    hg clone http://bitbucket.org/\$bitbucket_repo -b ${dep_branch} \
	$WORKSPACE/$dep 

    GENERIC_ENABLE_CPPCHECK=false
    GENERIC_ENABLE_TESTS=false 
    SOFTWARE_DIR=$dep
    . ${WORKSPACE}/${SCRIPT_DIR}/lib/_generic_linux_compilation_build.sh.bash
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
make -j${MAKE_JOBS}
echo '# END SECTION'

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
  echo '# END SECTION'
fi

# Only run cppcheck on trusty
if [ "$DISTRO" = "trusty" ]; then 
  echo '# BEGIN SECTION: running cppcheck'
  # Step 3: code check
  cd $WORKSPACE/gazebo
  sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
  echo '# END SECTION'
else
  mkdir -p $WORKSPACE/build/cppcheck_results/
  echo "<results></results>" >> $WORKSPACE/build/cppcheck_results/empty.xml 
fi
echo '# END SECTION'
DELIM

USE_OSRF_REPO=true
SOFTWARE_DIR="gazebo"
DEPENDENCY_PKGS="${BASE_DEPENDENCIES} \
                 ${GAZEBO_BASE_DEPENDENCIES} \
		 ${GAZEBO_EXTRA_DEPENDENCIES} \
		 ${EXTRA_PACKAGES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
