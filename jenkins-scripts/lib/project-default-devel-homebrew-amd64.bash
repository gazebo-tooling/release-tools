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
${RUN_DIR}/bin/brew install https://raw.githubusercontent.com/j-rivero/homebrew-protobuf/master/protobuf.rb
# Hack to use ffmepg with gazebo - Need to fix detection first
${RUN_DIR}/bin/brew install ffmpeg

# Step 2. Install dependencies of ${PROJECT}
${RUN_DIR}/bin/brew tap osrf/simulation

# Unlink system dependencies first
for dep in `/usr/local/bin/brew deps ${PROJECT}`
do
  /usr/local/bin/brew unlink ${dep} || true
done || true

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
rm -fr ${RUN_DIR}
