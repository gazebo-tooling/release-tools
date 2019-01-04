# Parameters
#  - WORKSPACE
#  - TIMING_DIR
#  - SOFTWARE_DIR: directory relative path to find sources
#  - GENERIC_ENABLE_TIMING (optional) [default true]
#  - GENERIC_ENABLE_CPPCHECK (optional) [default true] run cppcheck
#  - GENERIC_ENABLE_TESTS (optional) [default true] run tests
#  - BUILDING_EXTRA_CMAKE_PARAMS (optional) extra cmake params
#  - BUILD_<lib name> (optional) build dependency from source, for example, BUILD_IGN_MATH
#    - <lib name>_BRANCH (optional [default: default]) branch for BUILD_<lib_name>

if [[ -z ${SOFTWARE_DIR} ]]; then
    echo "SOFTWARE_DIR variable is unset. Please fix the code"
    exit 1
fi

[[ -z $GENERIC_ENABLE_TIMING ]] && GENERIC_ENABLE_TIMING=true
[[ -z $GENERIC_ENABLE_CPPCHECK ]] && GENERIC_ENABLE_CPPCHECK=true
[[ -z $GENERIC_ENABLE_TESTS ]] && GENERIC_ENABLE_TESTS=true

cat > build.sh << DELIM_HEADER
#!/bin/bash
set -ex

if $GENERIC_ENABLE_TIMING; then
  source ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}
fi
DELIM_HEADER

# Process the source build of dependencies if needed
OSRF_DEPS="IGN_CMAKE IGN_TOOLS IGN_MATH IGN_MSGS IGN_TRANSPORT IGN_COMMON IGN_FUEL_TOOLS SDFORMAT IGN_RENDERING IGN_SENSORS IGN_GUI IGN_GAZEBO"
OSRF_DEPS_DONE=""
for dep_uppercase in $OSRF_DEPS; do
  dep=`echo $dep_uppercase | tr '[:upper:]' '[:lower:]'`
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} mercurial"
  eval dependecy_installation="\$BUILD_$dep_uppercase"

  # Prevent multiple builds of same dep
  if grep -q "$dep_uppercase" <<< "$OSRF_DEPS_DONE"; then
    continue
  fi
  OSRF_DEPS_DONE="${OSRF_DEPS_DONE} $dep_uppercase"

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
echo '# BEGIN SECTION: configure'
# Step 2: configure and build
cd $WORKSPACE
[[ ! -d $WORKSPACE/build ]] && mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/${SOFTWARE_DIR} ${BUILDING_EXTRA_CMAKE_PARAMS} \
    -DCMAKE_INSTALL_PREFIX=/usr
echo '# END SECTION'

echo '# BEGIN SECTION: compiling'
init_stopwatch COMPILATION
make -j${MAKE_JOBS}
echo '# END SECTION'

echo '# BEGIN SECTION: installing'
make install
stop_stopwatch COMPILATION
echo '# END SECTION'

if $GENERIC_ENABLE_TESTS; then
  echo '# BEGIN SECTION: running tests'
  init_stopwatch TEST
  mkdir -p \$HOME
  make test ARGS="-VV" || true
  stop_stopwatch TEST
  echo '# END SECTION'
else
  echo "Requested: no test run"
fi

if $GENERIC_ENABLE_CPPCHECK; then
  echo '# BEGIN SECTION: cppcheck'
  cd $WORKSPACE/${SOFTWARE_DIR}
  if [ ! -f tools/cpplint_to_cppcheckxml.py ]; then
    mkdir -p tools
    cp $WORKSPACE/scripts/jenkins-scripts/tools/cpplint_to_cppcheckxml.py tools/
  fi
  init_stopwatch CPPCHECK
  sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
  stop_stopwatch CPPCHECK
  echo '# END SECTION'
else
  echo "Requested: no ccpcheck run"
fi
DELIM
