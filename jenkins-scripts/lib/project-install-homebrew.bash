#!/bin/bash -x
set -e

# Knowing Script dir beware of symlink
if [[ -z "${SCRIPT_DIR}" ]]; then
  [[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
  SCRIPT_DIR="${SCRIPT_DIR%/lib/*}"
fi

# Get bottle name as first argument to this script
BOTTLE_NAME=$1 # project will have the major version included (ex gazebo2)

export HOMEBREW_PREFIX=/usr/local
export HOMEBREW_CELLAR=${HOMEBREW_PREFIX}/Cellar
export PATH=${HOMEBREW_PREFIX}/bin:$PATH

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

echo "# BEGIN SECTION: install ${BOTTLE_NAME}"
brew install --include-test ${BOTTLE_NAME}

# add X11 path so glxinfo can be found
export PATH="${PATH}:/opt/X11/bin"

# set display before cmake
# search for Xquartz instance owned by current user
export DISPLAY=$(ps ax \
  | grep '[[:digit:]]*:[[:digit:]][[:digit:]].[[:digit:]][[:digit:]] /opt/X11/bin/Xquartz' \
  | grep "auth /Users/$(whoami)/" \
  | sed -e 's@.*Xquartz @@' -e 's@ .*@@'
)

echo "# BEGIN SECTION: run tests"
brew linkage --test ${BOTTLE_NAME}
if ! brew ruby -e "exit '${BOTTLE_NAME}'.f.test_defined?"; then
  # no test defined
  echo MARK_AS_UNSTABLE
  brew audit ${BOTTLE_NAME}
else
  brew test ${BOTTLE_NAME}
  brew audit --strict ${BOTTLE_NAME}
fi
echo '# END SECTION'

echo "#BEGIN SECTION: brew doctor analysis"
brew missing || brew install $(brew missing | awk '{print $2}') && brew missing
# if szip is installed, skip brew doctor
# remove this line when hdf5 stops depending on the deprecated szip formula
# https://github.com/Homebrew/homebrew-core/issues/96930
brew list | grep '^szip$' || brew doctor || echo MARK_AS_UNSTABLE
echo '# END SECTION'

echo "# BEGIN SECTION: re-add group write permissions"
chmod -R ug+rwx ${HOMEBREW_CELLAR}
echo '# END SECTION'
