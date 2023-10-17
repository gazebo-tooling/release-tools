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
export HOMEBREW_UPDATE_TO_TAG=1

# There might be a background process that blocks `brew update`, so we try to
# run it several times until it succeeds.
# See https://github.com/Homebrew/brew/issues/1155
brew_update_retry_count=0
until ${BREW_BINARY} update || (( brew_update_retry_count++ > 10 ))
do
  sleep 60
done
# manually exclude a ruby warning that jenkins thinks is from clang
# https://github.com/osrf/homebrew-simulation/issues/1343
${BREW_BINARY} install ${BREW_BASE_DEPENDCIES} \
   2>&1 | grep -v 'warning: conflicting chdir during another chdir block'
