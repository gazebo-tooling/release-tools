# Parameters
#  - WORKSPACE
#  - TIMING_DIR
#  - SOFTWARE_DIR: directory relative path to find sources
#  - GENERIC_ENABLE_TIMING (optional) [default true]
#  - GENERIC_ENABLE_CPPCHECK (optional) [default true] run cppcheck
#  - GENERIC_ENABLE_TESTS (optional) [default true] run tests
#  - BUILDING_EXTRA_CMAKE_PARAMS (optional) extra cmake params
#  - BUILDING_EXTRA_MAKETEST_PARAMS (optional) extra "make test ARGS=" params
#  - BUILD_<lib name> (optional) build dependency from source, for example, BUILD_GZ_MATH
#    - <lib name>_BRANCH (optional [default: master]) branch for BUILD_<lib_name>

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
OSRF_DEPS="GZ_CMAKE GZ_UTILS GZ_TOOLS GZ_MATH GZ_MSGS GZ_TRANSPORT GZ_COMMON GZ_FUEL_TOOLS SDFORMAT GZ_PHYSICS GZ_RENDERING GZ_SENSORS GZ_GUI GZ_SIM"
OSRF_DEPS_DONE=""
for dep_uppercase in $OSRF_DEPS; do
  dep=`echo $dep_uppercase | tr '[:upper:]' '[:lower:]'`
  DEPENDENCY_PKGS="${DEPENDENCY_PKGS} git"
  eval dependecy_installation="\$BUILD_$dep_uppercase"

  # Prevent multiple builds of same dep
  if grep -q "$dep_uppercase" <<< "$OSRF_DEPS_DONE"; then
    continue
  fi
  OSRF_DEPS_DONE="${OSRF_DEPS_DONE} $dep_uppercase"

  if [[ -n ${dependecy_installation} ]] && ${dependecy_installation}; then
      # Handle the depedency BRANCH
      eval dep_branch=\$$dep_uppercase\_BRANCH
      [[ -z ${dep_branch} ]] && dep_branch='master'
cat >> build.sh << DELIM_BUILD_DEPS
    echo "# BEGIN SECTION: building dependency: ${dep} (${dep_branch})"
    echo '# END SECTION'
    sudo rm -fr $WORKSPACE/$dep

    if [[ ${dep/ign} == ${dep} ]]; then
      dependency_repo="osrf/${dep}"
    else
      # need to replace _ by -
      dependency_repo="gazebosim/${dep//_/-}"
    fi

    git clone http://github.com/\$dependency_repo -b ${dep_branch} \
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
make -j\$(cat $WORKSPACE/make_jobs)
echo '# END SECTION'

echo '# BEGIN SECTION: installing'
sudo make install
stop_stopwatch COMPILATION
echo '# END SECTION'

if $GENERIC_ENABLE_TESTS; then
  echo '# BEGIN SECTION: running tests'
  init_stopwatch TEST
  mkdir -p \$HOME
  make test ARGS="-VV ${BUILDING_EXTRA_MAKETEST_PARAMS} --output-junit cmake_junit_output.xml" || true
  if [ -f cmake_junit_output.xml ]; then
    python3 $WORKSPACE/scripts/jenkins-scripts/tools/cmake_to_gtest_junit_output.py cmake_junit_output.xml test_results  || true
  fi
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
