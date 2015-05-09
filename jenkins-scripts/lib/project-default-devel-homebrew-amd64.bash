#!/bin/bash -x
set -e

export HOMEBREW_MAKE_JOBS=${MAKE_JOBS}

# Get project name as first argument to this script
PROJECT=$1
PROJECT_ARGS=${2}

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Step 1. Set up homebrew
echo '# BEGIN SECTION: clean up /usr/local'
sudo chown -R jenkins /usr/local 
cd /usr/local && git clean -fdx
echo '# END SECTION'

echo '# BEGIN SECTION: install latest homebrew'
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
echo '# END SECTION'

echo '# BEGIN SECTION: brew information'
# Run brew update to get latest versions of formulae
brew update
# Run brew config to print system information
brew config
# Run brew doctor to check for problems with the system
brew doctor || true
echo '# END SECTION'

echo '# BEGIN SECTION: setup the osrf/simulation tap'
brew tap osrf/simulation
echo '# END SECTION'

IS_A_HEAD_FORMULA=${IS_A_HEAD_PROJECT:-false}
HEAD_STR=""
if $IS_A_HEAD_PROJECT; then
    HEAD_STR="--HEAD"
fi

echo "# BEGIN SECTION: install ${PROJECT} dependencies"
# Process the package dependencies
# Run twice! details about why in:
# https://github.com/osrf/homebrew-simulation/pull/18#issuecomment-45041755 
brew install ${HEAD_STR} ${PROJECT} ${PROJECT_ARGS} --only-dependencies
brew install ${HEAD_STR} ${PROJECT} ${PROJECT_ARGS} --only-dependencies
echo '# END SECTION'

echo "# BEGIN SECTION: configuring ${PROJECT}"
# Step 3. Manually compile and install ${PROJECT}
cd ${WORKSPACE}/${PROJECT}
# Need the sudo since the test are running with roots perms to access to GUI
sudo rm -fr ${WORKSPACE}/build
mkdir -p ${WORKSPACE}/build
cd ${WORKSPACE}/build
 
# add X11 path so glxinfo can be found
export PATH="${PATH}:/opt/X11/bin"

# set display before cmake
# search for Xquartz instance owned by jenkins
export DISPLAY=$(ps ax \
  | grep '[[:digit:]]*:[[:digit:]][[:digit:]].[[:digit:]][[:digit:]] /opt/X11/bin/Xquartz' \
  | grep 'auth /Users/jenkins/' \
  | sed -e 's@.*Xquartz @@' -e 's@ .*@@'
)

cmake ${WORKSPACE}/${PROJECT} \
      -DCMAKE_INSTALL_PREFIX=${RUN_DIR}/Cellar/${PROJECT}/HEAD \
      -DCMAKE_PREFIX_PATH=${RUN_DIR} \
      -DBOOST_ROOT=${RUN_DIR}
echo '# END SECTION'

echo "# BEGIN SECTION: compile ${PROJECT}"
make -j${MAKE_JOBS} install
brew link ${PROJECT}
brew doctor
echo '# END SECTION'

echo "# BEGIN SECTION: run tests"
# Need to clean up models before run tests (issue 27)
rm -fr \$HOME/.gazebo/models

cd $WORKSPACE/build/
# May need sudo to run tests?
make test ARGS="-VV" || true
echo '# END SECTION'

# Step 5. Clean up
rm -fr ${RUN_DIR}
echo '# END SECTION'
