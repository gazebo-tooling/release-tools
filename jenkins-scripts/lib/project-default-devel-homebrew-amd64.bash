#!/bin/bash -x
set -e

# Knowing Script dir beware of symlink
if [[ -z "${SCRIPT_DIR}" ]]; then
  [[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
  SCRIPT_DIR="${SCRIPT_DIR%/lib/*}"
fi

export HOMEBREW_MAKE_JOBS=${MAKE_JOBS}

# Get project name as first argument to this script
PROJECT=$1 # project will have the major version included (ex gazebo2)
PROJECT_ARGS=${2}
# PROJECT_PATH can be passed as env variable or assume that is the same than project name
PROJECT_PATH=${:-PROJECT}
# Check for major version number
# the PROJECT_FORMULA variable is only used for dependency resolution
PROJECT_FORMULA=${PROJECT//[0-9]}$(\
  python3 ${SCRIPT_DIR}/tools/detect_cmake_major_version.py \
  ${WORKSPACE}/${PROJECT_PATH}/CMakeLists.txt || true)

export HOMEBREW_PREFIX=/usr/local
export HOMEBREW_CELLAR=${HOMEBREW_PREFIX}/Cellar
export PATH=${HOMEBREW_PREFIX}/bin:${HOMEBREW_PREFIX}/sbin:$PATH

export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python

# make verbose mode?
MAKE_VERBOSE_STR=""
if [[ ${MAKE_VERBOSE} ]]; then
  MAKE_VERBOSE_STR="VERBOSE=1"
fi

# Step 1. Set up homebrew
echo "# BEGIN SECTION: clean up ${HOMEBREW_PREFIX}"
export HOMEBREW_NO_INSTALL_FROM_API=1
. ${SCRIPT_DIR}/lib/_homebrew_cleanup.bash
. ${SCRIPT_DIR}/lib/_homebrew_base_setup.bash
brew cleanup || echo "brew cleanup couldn't be run"
mkdir -p ${HOMEBREW_CELLAR}
chmod -R ug+rwx ${HOMEBREW_CELLAR}
echo '# END SECTION'

echo '# BEGIN SECTION: brew information'
# Run brew update to get latest versions of formulae
brew update
# Don't let brew auto-update any more for this session
# to ensure consistency
export HOMEBREW_NO_AUTO_UPDATE=1
# Run brew config to print system information
brew config
# Run brew doctor to check for problems with the system
brew doctor || echo MARK_AS_UNSTABLE
echo '# END SECTION'

echo '# BEGIN SECTION: setup the osrf/simulation tap'
brew tap osrf/simulation
echo '# END SECTION'

if [[ -n "${ghprbSourceBranch}" ]] && \
   python3 ${SCRIPT_DIR}/tools/detect_ci_matching_branch.py "${ghprbSourceBranch}"
then
  echo "# BEGIN SECTION: trying to checkout branch ${ghprbSourceBranch} from osrf/simulation"
  pushd $(brew --repo osrf/simulation)
  git fetch origin ${ghprbSourceBranch} || true
  git checkout ${ghprbSourceBranch} || true
  popd
  echo '# END SECTION'
fi

echo "# BEGIN SECTION: install ${PROJECT_FORMULA} dependencies"
# Process the package dependencies
brew install ${PROJECT_FORMULA} ${PROJECT_ARGS} --only-dependencies
# the following is needed to install :build dependencies of a formula
brew install $(brew deps --1 --include-build ${PROJECT_FORMULA})

# pytest is needed to run python tests with junit xml output
PIP_PACKAGES_NEEDED="${PIP_PACKAGES_NEEDED} pytest"

if [[ "${RERUN_FAILED_TESTS}" -gt 0 ]]; then
  # Install lxml for flaky_junit_merge.py
  PIP_PACKAGES_NEEDED="${PIP_PACKAGES_NEEDED} lxml"
fi

if [[ -n "${PIP_PACKAGES_NEEDED}" ]]; then
  brew install python
  PIP=pip3
  if ! which ${PIP}; then
    PIP=/usr/local/opt/python/bin/pip3
  fi
  ${PIP} install ${PIP_PACKAGES_NEEDED}
fi

if [[ -z "${DISABLE_CCACHE}" ]]; then
  brew install ccache
  export PATH=/usr/local/opt/ccache/libexec:$PATH
fi
echo '# END SECTION'

# Step 3. Manually compile and install ${PROJECT}
echo "# BEGIN SECTION: configure ${PROJECT}"
cd ${WORKSPACE}/${PROJECT_PATH}
rm -fr ${WORKSPACE}/build
mkdir -p ${WORKSPACE}/build
cd ${WORKSPACE}/build

# add X11 path so glxinfo can be found
export PATH="${PATH}:/opt/X11/bin"

# set display before cmake
# search for Xquartz instance owned by current user
export DISPLAY=$(ps ax \
  | grep '[[:digit:]]*:[[:digit:]][[:digit:]].[[:digit:]][[:digit:]] /opt/X11/bin/Xquartz' \
  | grep "auth /Users/$(whoami)/" \
  | sed -e 's@.*Xquartz @@' -e 's@ .*@@'
)

CMAKE_ARGS=""
# set CMAKE_PREFIX_PATH if we are using qt@5
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'qt@5' }.empty?"; then
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/qt@5
fi
# set cmake args if we are using qwt-qt5
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'qwt-qt5' }.empty?"; then
  CMAKE_ARGS='-DQWT_WIN_INCLUDE_DIR=/usr/local/opt/qwt-qt5/lib/qwt.framework/Headers -DQWT_WIN_LIBRARY_DIR=/usr/local/opt/qwt-qt5/lib'
fi
# Workaround for cmake@3.21.4: set PATH
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'osrf/simulation/cmake@3.21.4' }.empty?"; then
  export PATH=/usr/local/opt/cmake@3.21.4/bin:${PATH}
fi
# Workaround for ffmpeg 4: set PKG_CONFIG_PATH if we are using ffmpeg@4
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'osrf/simulation/ffmpeg@4' }.empty?"; then
  export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/ffmpeg@4/lib/pkgconfig
fi
# Workaround for tbb@2020_u3: set CPATH, LIBRARY_PATH, and CMAKE_PREFIX_PATH
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'osrf/simulation/tbb@2020_u3' }.empty?"; then
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/tbb@2020_u3
  export CPATH=${CPATH}:/usr/local/opt/tbb@2020_u3/include
  export LIBRARY_PATH=${LIBRARY_PATH}:/usr/local/opt/tbb@2020_u3/lib
fi
# if we are using gts, need to add gettext library path since it is keg-only
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'gettext' }.empty?"; then
  export LIBRARY_PATH=${LIBRARY_PATH}:/usr/local/opt/gettext/lib
fi
# if we are using boost, need to add icu4c library path since it is keg-only
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'icu4c' }.empty?"; then
  export LIBRARY_PATH=${LIBRARY_PATH}:/usr/local/opt/icu4c/lib
fi

# if we are using dart@6.10.0 (custom OR port), need to add dartsim library path since it is keg-only
if brew ruby -e "exit ! '${PROJECT_FORMULA}'.f.recursive_dependencies.map(&:name).keep_if { |d| d == 'osrf/simulation/dartsim@6.10.0' }.empty?"; then
  export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/dartsim@6.10.0
  export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/usr/local/opt/dartsim@6.10.0/lib:/usr/local/opt/octomap/local
  export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/dartsim@6.10.0/lib/pkgconfig
fi

cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_INSTALL_PREFIX=/usr/local/Cellar/${PROJECT_FORMULA}/HEAD \
     ${CMAKE_ARGS} \
     ${WORKSPACE}/${PROJECT_PATH}
echo '# END SECTION'

echo "# BEGIN SECTION: compile and install ${PROJECT_FORMULA}"
make -j${MAKE_JOBS} ${MAKE_VERBOSE_STR} install
brew link ${PROJECT_FORMULA}
echo '# END SECTION'

echo "#BEGIN SECTION: brew doctor analysis"
brew missing || brew install $(brew missing | awk '{print $2}') && brew missing
# if szip is installed, skip brew doctor
# remove this line when hdf5 stops depending on the deprecated szip formula
# https://github.com/Homebrew/homebrew-core/issues/96930
brew list | grep '^szip$' || brew doctor || echo MARK_AS_UNSTABLE
echo '# END SECTION'

# CHECK PRE_TESTS_EXECUTION_HOOK AND RUN
# expr length is not portable. wc -c, returns 1 on empty str
if [ `echo "${PRE_TESTS_EXECUTION_HOOK}" | wc -c` -gt 1 ]; then
  # to be able to handle hooks in a pure multiline form, this dirty hack
  TMPFILE_HOOK=$(mktemp /tmp/.brew_pretesthook_XXXX)
  cat > ${TMPFILE_HOOK} <<-DELIM
  ${PRE_TESTS_EXECUTION_HOOK}
DELIM
  . ${TMPFILE_HOOK}
  rm ${TMPFILE_HOOK}
fi

echo "# BEGIN SECTION: run tests"
# Need to clean up models before run tests (issue 27)
rm -fr \$HOME/.gazebo/models test_results*

# Run `make test`
# If it has any failures, then rerun the failed tests one time
# and merge the junit results
. ${WORKSPACE}/scripts/jenkins-scripts/lib/make_test_rerun_failed.bash
echo '# END SECTION'

echo "# BEGIN SECTION: re-add group write permissions"
chmod -R ug+rwx ${HOMEBREW_CELLAR}
echo '# END SECTION'
