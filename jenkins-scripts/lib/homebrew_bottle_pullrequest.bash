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
git config --global user.name "OSRF Build Bot"
git config --global user.email "osrfbuild@osrfoundation.org"
cat ${HOME}/.gitconfig
brew test-bot             \
    --tap=osrf/simulation \
    --bottle              \
    --ci-pr               \
    --verbose ${PULL_REQUEST_URL}
echo '# END SECTION'

echo '# BEGIN SECTION: export bottle'
mv *.bottle.tar.gz ${PKG_DIR}
mv *.bottle.rb ${PKG_DIR}
echo '# END SECTION'
