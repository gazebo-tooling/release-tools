#/bin/bash +x
set -e

BREW_BINARY_DIR=/usr/local/bin
BREW_BINARY=${BREW_BINARY_DIR}/brew

# Restore the basic stuff
if [[ -n ${SCRIPT_LIBDIR} ]]; then
  . ${SCRIPT_LIBDIR}/dependencies_archive.sh
elif [[ -n ${SCRIPT_DIR} ]]; then
  . ${SCRIPT_DIR}/lib/dependencies_archive.sh
else
  echo "Can not find the dependencies_archive.sh"
  exit -1
fi

git -C $(${BREW_BINARY} --repo) fsck
# don't use HOMEBREW_UPDATE_TO_TAG until brew 3.3.13 is released
# due to https://github.com/Homebrew/brew/issues/12788
unset HOMEBREW_UPDATE_TO_TAG
${BREW_BINARY} update
# manually exclude a ruby warning that jenkins thinks is from clang
# https://github.com/osrf/homebrew-simulation/issues/1343
${BREW_BINARY} install ${BREW_BASE_DEPENDCIES} \
   2>&1 | grep -v 'warning: conflicting chdir during another chdir block'
