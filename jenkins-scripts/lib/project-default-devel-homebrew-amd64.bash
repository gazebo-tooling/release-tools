#!/bin/bash -x
set -e

export HOMEBREW_MAKE_JOBS=${MAKE_JOBS}

# Get project name as first argument to this script
PROJECT=$1

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

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
# Hack to use a patched version of protobuf
${RUN_DIR}/bin/brew tap j-rivero/simulation
${RUN_DIR}/bin/brew install j-rivero/simulation/protobuf
# Hack to use ffmepg with gazebo - Need to fix detection first
${RUN_DIR}/bin/brew install ffmpeg

# Step 2. Install dependencies of ${PROJECT}
${RUN_DIR}/bin/brew tap osrf/simulation

# If the case of gazebo, reuse qt so we don't need to compile it all the time
if [[ $PROJECT == 'gazebo' ]]; then
  if [[ ! $(find ${LOCAL_CELLAR} -name qt-4.*.mavericks.bottle.tar.gz) ]]; then
    curl -L -o "${LOCAL_CELLAR}/qt-4.8.5.mavericks.bottle.tar.gz" \
      https://www.dropbox.com/s/to19m8jw6elk9m0/qt-4.8.5.mavericks.bottle.tar.gz
  fi

  ${RUN_DIR}/bin/brew install "${LOCAL_CELLAR}/qt-4.8.5.mavericks.bottle.tar.gz"

  # The bottle has some hardcoded files in qmake configurations. Hack them.
  # see https://bitbucket.org/osrf/release-tools/pull-request/30
  rm -fr ${HOME}/jenkins.R7cR
  ln -s ${RUN_DIR} ${HOME}/jenkins.R7cR 
fi
# Process the package dependencies
${RUN_DIR}/bin/brew install ${PROJECT} --only-dependencies

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

${RUN_DIR}/bin/cmake ${WORKSPACE}/${PROJECT} \
      -DCMAKE_INSTALL_PREFIX=${RUN_DIR}/Cellar/${PROJECT}/HEAD \
      -DCMAKE_PREFIX_PATH=${RUN_DIR} \
      -DBOOST_ROOT=${RUN_DIR}

make -j${MAKE_JOBS} install
${RUN_DIR}/bin/brew link ${PROJECT}

# Need to use root to access to the graphical env
export DISPLAY=$(sudo find /private/tmp -name *xquartz* | sed 's:/private::')

cat > test_run.sh << DELIM
cd $WORKSPACE/build/
export PKG_CONFIG_PATH=${RUN_DIR}/lib/pkgconfig
export DYLD_FALLBACK_LIBRARY_PATH=${RUN_DIR}/lib
export BOOST_ROOT=${RUN_DIR}
export PATH="${PATH}:${RUN_DIR}/bin"
export CMAKE_PREFIX_PATH=${RUN_DIR}

make test ARGS="-VV" || true
DELIM

chmod +x test_run.sh
sudo  ./test_run.sh

# Step 5. Clean up
#rm -fr ${RUN_DIR}
