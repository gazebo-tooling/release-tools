#!/bin/bash -x
set -e

export HOMEBREW_MAKE_JOBS=${MAKE_JOBS}

# Get project name as first argument to this script
PROJECT=$1
PROJECT_ARGS=${2}

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# make verbose mode?
MAKE_VERBOSE_STR=""
if [[ ${MAKE_VERBOSE} ]]; then
  MAKE_VERBOSE_STR="VERBOSE=1"
fi

# Step 1. Set up homebrew
RUN_DIR=$(mktemp -d ${HOME}/jenkins.XXXX)
echo "Install into: ${RUN_DIR}"
cd $RUN_DIR
# Install homebrew
curl -L -o homebrew.tar.gz https://github.com/Homebrew/homebrew/tarball/master 
tar --strip 1 -xzf homebrew.tar.gz

# Need to create cache so the system one (without permissions) is not used
LOCAL_CELLAR=${HOME}/Library/Caches/Homebrew
mkdir -p ${LOCAL_CELLAR}

# Run brew update to get latest versions of formulae
${RUN_DIR}/bin/brew update

# Run brew config to print system information
${RUN_DIR}/bin/brew config

# Run brew doctor to check for problems with the system
${RUN_DIR}/bin/brew doctor || true

# Step 2. Install dependencies of ${PROJECT}
${RUN_DIR}/bin/brew tap osrf/simulation

# Unlink system dependencies first
for dep in `/usr/local/bin/brew deps ${PROJECT} ${PROJECT_ARGS}`
do
  /usr/local/bin/brew unlink ${dep} || true
done || true

IS_A_HEAD_FORMULA=${IS_A_HEAD_PROJECT:-false}
HEAD_STR=""
if $IS_A_HEAD_PROJECT; then
    HEAD_STR="--HEAD"
fi

# Process the package dependencies
# Run twice! details about why in:
# https://github.com/osrf/homebrew-simulation/pull/18#issuecomment-45041755 
${RUN_DIR}/bin/brew install ${HEAD_STR} ${PROJECT} ${PROJECT_ARGS} --only-dependencies
${RUN_DIR}/bin/brew install ${HEAD_STR} ${PROJECT} ${PROJECT_ARGS} --only-dependencies

# Step 3. Manually compile and install ${PROJECT}
cd ${WORKSPACE}/${PROJECT}
# Need the sudo since the test are running with roots perms to access to GUI
sudo rm -fr ${WORKSPACE}/build
mkdir -p ${WORKSPACE}/build
cd ${WORKSPACE}/build
 

# Mimic the homebrew variables
export PKG_CONFIG_PATH=${RUN_DIR}/lib/pkgconfig
export DYLD_FALLBACK_LIBRARY_PATH="$DYLD_FALLBACK_LIBRARY_PATH:${RUN_DIR}/lib"
export PATH="${PATH}:${RUN_DIR}/bin"
export C_INCLUDE_PATH="${C_INCLUDE_PATH}:${RUN_DIR}/include"
export CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:${RUN_DIR}/include"

# add X11 path so glxinfo can be found
export PATH="${PATH}:/opt/X11/bin"

# set display before cmake
# search for Xquartz instance owned by jenkins
export DISPLAY=$(ps ax \
  | grep '[[:digit:]]*:[[:digit:]][[:digit:]].[[:digit:]][[:digit:]] /opt/X11/bin/Xquartz' \
  | grep 'auth /Users/jenkins/' \
  | sed -e 's@.*Xquartz @@' -e 's@ .*@@'
)

${RUN_DIR}/bin/cmake ${WORKSPACE}/${PROJECT} \
      -DCMAKE_INSTALL_PREFIX=${RUN_DIR}/Cellar/${PROJECT}/HEAD \
      -DCMAKE_PREFIX_PATH=${RUN_DIR} \
      -DBOOST_ROOT=${RUN_DIR}

make -j${MAKE_JOBS} ${MAKE_VERBOSE_STR} install
${RUN_DIR}/bin/brew link ${PROJECT}

cat > test_run.sh << DELIM
cd $WORKSPACE/build/
export PKG_CONFIG_PATH=${RUN_DIR}/lib/pkgconfig
export DYLD_FALLBACK_LIBRARY_PATH=${RUN_DIR}/lib
export BOOST_ROOT=${RUN_DIR}
export PATH="${PATH}:${RUN_DIR}/bin"
export CMAKE_PREFIX_PATH=${RUN_DIR}

# Need to clean up models before run tests (issue 27)
rm -fr \$HOME/.gazebo/models
make test ARGS="-VV" || true
DELIM

chmod +x test_run.sh
sudo  ./test_run.sh

# Step 5. Clean up
rm -fr ${RUN_DIR}
