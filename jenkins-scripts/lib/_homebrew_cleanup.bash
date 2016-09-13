#/bin/bash +x
set -e

BREW_BINARY_DIR=/usr/local/bin
BREW_BINARY=${BREW_BINARY_DIR}/brew

# Clear all installed homebrew packages, links, taps, and kegs
${BREW_BINARY} remove --force $(${BREW_BINARY} list)
rm -rf /usr/local/lib/python2.7/site-packages
${BREW_BINARY} untap $(${BREW_BINARY} tap)

# Restore the basic stuff
if [[ -n ${SCRIPT_LIBDIR} ]]; then
  . ${SCRIPT_LIBDIR}/dependencies_archive.sh
elif [[ -n ${SCRIPT_DIR} ]]; then
  . ${SCRIPT_DIR}/lib/dependencies_archive.sh
else
  echo "Can not find the dependencies_archive.sh"
  exit -1
fi

${BREW_BINARY} update
${BREW_BINARY} install ${BREW_BASE_DEPENDCIES}

# test-bot needs variables and does not work just with config not sure why
export GIT_AUTHOR_NAME="OSRF Build Bot"
export GIT_COMMITTER_NAME=${GIT_AUTHOR_NAME}
export GIT_AUTHOR_EMAIL="osrfbuild@osrfoundation.org"
export GIT_COMMITTER_EMAIL=${GIT_AUTHOR_EMAIL}
git config --global user.name "${GIT_AUTHOR_NAME}"
git config --global user.email "${GIT_AUTHOR_EMAIL}"
