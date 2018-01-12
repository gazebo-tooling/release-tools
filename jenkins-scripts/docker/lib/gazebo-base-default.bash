#!/bin/bash -x

# Parameters
#  - GAZEBO_BASE_CMAKE_ARGS (optional) extra arguments to pass to cmake
#  - GAZEBO_BASE_TESTS_HOOK (optional) [default to run UNIT, INTEGRATION, REGRESSION, EXAMPLE]
#                           piece of code to run in the testing section
#  - GAZEBO_BUILD_$DEP      (optional) [default false]
#                           build dependencies from source.
#                           DEP = SDFORMAT | IGN_MATH | IGN_TRANSPORT | IGN_GUI | IGN_COMMON
#                           branch parameter = $DEP_BRANCH

#stop on error
set -e

GAZEBO_OSRF_DEPS="SDFORMAT IGN_MATH IGN_TRANSPORT IGN_GUI IGN_COMMON"

. ${SCRIPT_DIR}/lib/_gazebo_version_hook.bash

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="gazebo_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

[ -z ${COVERAGE_ENABLED} ] && COVERAGE_ENABLED=false

# If Coverage build type was supplied in GAZEBO_BASE_CMAKE_ARGS, add lcov
# package.
if [[ ${GAZEBO_BASE_CMAKE_ARGS} != ${GAZEBO_BASE_CMAKE_ARGS/Coverage} ]]; then
  EXTRA_PACKAGES="${EXTRA_PACKAGES} lcov"
fi

if ${COVERAGE_ENABLED} ; then
  # Workaround on problem with setting HOME to /var/lib/jenkins
  if [[ -f $HOME/bullseye-jenkins-license ]]; then
      LICENSE_FILE="$HOME/bullseye-jenkins-license"
  else
      LICENSE_FILE="/var/lib/jenkins/bullseye-jenkins-license"
  fi

  set +x # keep password secret
  BULLSEYE_LICENSE=`cat $LICENSE_FILE`
  set -x # back to debug

  EXTRA_PACKAGES="${EXTRA_PACKAGES} wget"
fi

if [[ $GAZEBO_MAJOR_VERSION -lt 8 ]]; then
  GAZEBO_BASE_CMAKE_ARGS="${GAZEBO_BASE_CMAKE_ARGS} -DENABLE_TESTS_COMPILATION=True"
fi

SOFTWARE_DIR="gazebo"
if [ "${GAZEBO_EXPERIMENTAL_BUILD}" = true ]; then
  SOFTWARE_DIR="${SOFTWARE_DIR}_experimental"
fi

cat > build.sh << DELIM_DART
###################################################
# Make project-specific changes here
#
set -ex
source ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}

if ${COVERAGE_ENABLED} ; then
  echo '# BEGIN SECTION: setup bulleyes coverage'
  # Clean previous content
  rm -fr $WORKSPACE/coverage
  # Download and install Bullseyes
  cd $WORKSPACE
  rm -fr $WORKSPACE/Bulls*
  
  # Look for current version. NOT IN USE since we lost the maintenance support on 2014 
  # reenable if the support is back.
  # wget http://www.bullseye.com/download/ -O bull_index.html
  # BULL_TAR=\$( grep -R BullseyeCoverage-.*-Linux-x64.tar bull_index.html | head -n 1 | sed 's/.*">//' | sed 's/<.*//' )
  # wget http://www.bullseye.com/download/\$BULL_TAR -O bullseye.tar

  # Download package
  wget https://www.dropbox.com/s/i1ay7t8sg8i77jr/bullseye-8.8.9.tar -O bullseye.tar
  tar -xf bullseye.tar
  cd Bulls*
  # Set up the license
  echo $PATH >install-path
  rm -fr /usr/bullseyes
  set +x # keep password secret
  ./install --prefix /usr/bullseyes --key $BULLSEYE_LICENSE
  set -x # back to debug
  # Set up Bullseyes for compiling
  export PATH=/usr/bullseyes/bin:\$PATH
  export COVFILE=$WORKSPACE/gazebo/test.cov
  cd $WORKSPACE/gazebo
  covselect --file test.cov --add .
  cov01 --on
  echo '# END SECTION'
fi

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

    GENERIC_ENABLE_TIMING=false
    GENERIC_ENABLE_CPPCHECK=false
    GENERIC_ENABLE_TESTS=false
    SOFTWARE_DIR=$dep
    cd $WORKSPACE
    . ${SCRIPT_DIR}/lib/_generic_linux_compilation.bash ${SCRIPT_DIR}
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
  $WORKSPACE/${SOFTWARE_DIR}
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
  RERUN_FAILED_TESTS=1
  init_stopwatch TEST
  echo '# BEGIN SECTION: UNIT testing'
  make test ARGS="-VV -R UNIT_*" || true
  echo '# END SECTION'
  echo '# BEGIN SECTION: INTEGRATION testing'
  . ${WORKSPACE}/scripts/jenkins-scripts/lib/make_test_rerun_failed.bash "-VV -R INTEGRATION_*"
  echo '# END SECTION'
  echo '# BEGIN SECTION: REGRESSION testing'
  . ${WORKSPACE}/scripts/jenkins-scripts/lib/make_test_rerun_failed.bash "-VV -R REGRESSION_*"
  echo '# END SECTION'
  echo '# BEGIN SECTION: EXAMPLE testing'
  make test ARGS="-VV -R EXAMPLE_*" || true
  stop_stopwatch TEST
  echo '# END SECTION'
fi

if ${COVERAGE_ENABLED} ; then
  echo '# BEGIN SECTION: UNIT testing'
  rm -fr $WORKSPACE/coverage
  rm -fr $WORKSPACE/bullshtml
  mkdir -p $WORKSPACE/coverage
  covselect --add '!$WORKSPACE/build/' '!../build/' '!test/' '!tools/test/' '!deps/' '!/opt/' '!gazebo/rendering/skyx/' '!/tmp/'
  covhtml --srcdir $WORKSPACE/gazebo/ $WORKSPACE/coverage
  # Generate valid cover.xml file using the bullshtml software
  # java is needed to run bullshtml
  apt-get install -y default-jre
  cd $WORKSPACE
  wget http://bullshtml.googlecode.com/files/bullshtml_1.0.5.tar.gz -O bullshtml.tar.gz
  tar -xzf bullshtml.tar.gz
  cd bullshtml
  sh bullshtml .
  # Hack to remove long paths from report
  find . -name '*.html' -exec sed -i -e 's:${WORKSPACE}::g' {} \;
fi

echo '# BEGIN SECTION: running cppcheck'
init_stopwatch CPPCHECK
# Step 3: code check
cd $WORKSPACE/${SOFTWARE_DIR}
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
stop_stopwatch CPPCHECK
echo '# END SECTION'
DELIM

USE_OSRF_REPO=true
DEPENDENCY_PKGS="${BASE_DEPENDENCIES} \
                 ${GAZEBO_BASE_DEPENDENCIES} \
		 ${GAZEBO_EXTRA_DEPENDENCIES} \
		 ${EXTRA_PACKAGES}"

[[ -z ${GAZEBO_BUILD_IGN_MATH} ]] && GAZEBO_BUILD_IGN_MATH=false
if $GAZEBO_BUILD_IGN_MATH; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} ${IGN_MATH_DEPENDENCIES}"
fi

[[ -z ${GAZEBO_BUILD_IGN_MSGS} ]] && GAZEBO_BUILD_IGN_MSGS=false
if $GAZEBO_BUILD_IGN_MSGS; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} ${IGN_MSGS_DEPENDENCIES}"
fi

[[ -z ${GAZEBO_BUILD_IGN_TRANSPORT} ]] && GAZEBO_BUILD_IGN_TRANSPORT=false
if $GAZEBO_BUILD_IGN_TRANSPORT; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} ${IGN_TRANSPORT_DEPENDENCIES}"
fi

[[ -z ${GAZEBO_BUILD_IGN_GUI} ]] && GAZEBO_BUILD_IGN_GUI=false
if $GAZEBO_BUILD_IGN_GUI; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} ${IGN_GUI_DEPENDENCIES}"
fi

[[ -z ${GAZEBO_BUILD_IGN_COMMON} ]] && GAZEBO_BUILD_IGN_COMMON=false
if $GAZEBO_BUILD_IGN_COMMON; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} ${IGN_COMMON_DEPENDENCIES}"
fi

[[ -z ${GAZEBO_BUILD_SDFORMAT} ]] && GAZEBO_BUILD_SDFORMAT=false
if $GAZEBO_BUILD_SDFORMAT; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} ${SDFORMAT_BASE_DEPENDENCIES}"
fi

# Need for cmake DISPLAY check (it uses xwininfo command)
if [[ $USE_GPU_DOCKER ]]; then
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} x11-utils"
fi

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
