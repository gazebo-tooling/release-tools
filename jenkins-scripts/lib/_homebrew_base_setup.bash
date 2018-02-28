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
# disable update to tag for now since `brew ruby` is broken
#export HOMEBREW_UPDATE_TO_TAG=1
unset HOMEBREW_UPDATE_TO_TAG
${BREW_BINARY} update
${BREW_BINARY} install ${BREW_BASE_DEPENDCIES}
