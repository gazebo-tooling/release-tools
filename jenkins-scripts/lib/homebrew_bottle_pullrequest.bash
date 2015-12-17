#!/bin/bash -x
set -e

[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

PKG_DIR=${WORKSPACE}/pkgs

echo '# BEGIN SECTION: check variables'
if [ -z "${PULL_REQUEST_NUMBER}" ]; then
    echo PULL_REQUEST_NUMBER not specified
    exit -1
fi
echo '# END SECTION'

echo '# BEGIN SECTION: clean up environment'
rm -fr ${PKG_DIR} && mkdir -p ${PKG_DIR}
. ${SCRIPT_LIBDIR}/_homebrew_cleanup.bash
echo '# END SECTION'

echo '# BEGIN SECTION: run test-bot'
brew test-bot             \
    --tap=osrf/simulation \
    --bottle              \
    --ci-pr               \
    --verbose https://github.com/osrf/homebrew-simulation/pull/${PULL_REQUEST_NUMBER}
echo '# END SECTION'

echo '# BEGIN SECTION: export bottle'
mv *.bottle.tar.gz ${PKG_DIR}
mv *.bottle.rb ${PKG_DIR}
echo '# END SECTION'
