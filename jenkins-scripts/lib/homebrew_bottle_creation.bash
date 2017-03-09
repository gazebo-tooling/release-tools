#!/bin/bash -x
set -e

[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

export PATH="/usr/local/bin:$PATH"

PKG_DIR=${WORKSPACE}/pkgs

echo '# BEGIN SECTION: check variables'
if [ -z "${PULL_REQUEST_URL}" ]; then
    echo PULL_REQUEST_URL not specified
    exit -1
fi
echo '# END SECTION'

echo '# BEGIN SECTION: clean up environment'
rm -fr ${PKG_DIR} && mkdir -p ${PKG_DIR}
. ${SCRIPT_LIBDIR}/_homebrew_cleanup.bash
echo '# END SECTION'

echo '# BEGIN SECTION: run test-bot'
# The test-bot makes a full cleanup of all installed pkgs. Be sure of install back
# mercurial to keep the slave working
export HOMEBREW_DEVELOPER=1
brew test-bot --tap=osrf/simulation \
              --ci-pr ${PULL_REQUEST_URL} \
            || { brew install hg; exit -1; }
brew install hg
echo '# END SECTION'

echo '# BEGIN SECTION: export bottle'
if [[ $(find . -name '*.bottle.*' | wc -l | sed 's/^ *//') -lt 2 ]]; then
  echo "Can not find at least two bottle files. Something went wrong."
  exit -1
fi

mv *.bottle*.tar.gz ${PKG_DIR}
mv *.bottle.json ${PKG_DIR}

echo '# END SECTION'
