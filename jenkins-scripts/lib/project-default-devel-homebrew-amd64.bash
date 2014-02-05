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
mkdir -p ${HOME}/Library/Caches/Homebrew

# Step 2. Install dependencies of ${PROJECT}
${RUN_DIR}/bin/brew tap osrf/simulation
${RUN_DIR}/bin/brew install ${PROJECT} --only-dependencies

# Step 3. Manually compile and install ${PROJECT}
cd ${WORKSPACE}/${PROJECT}
mkdir -p ${WORKSPACE}/build
cd ${WORKSPACE}/build

# Need to trick config path
export PKG_CONFIG_PATH=${RUN_DIR}

${RUN_DIR}/bin/cmake ${WORKSPACE}/${PROJECT} \
      -DCMAKE_INSTALL_PREFIX=${RUN_DIR}/Cellar/${PROJECT}/HEAD \
      -DCMAKE_PREFIX_PATH=${RUN_DIR}

make -j${MAKE_JOBS} install
${RUN_DIR}/bin/brew link ${PROJECT}

# Step 4. Testing
make test ARGS="-VV" || true

# Step 5. Clean up
rm -fr ${RUN_DIR}
