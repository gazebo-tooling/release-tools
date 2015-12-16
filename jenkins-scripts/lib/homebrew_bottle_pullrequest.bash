#!/bin/bash -x

[[ -L ${0} ]] && SCRIPT_LIBDIR_DIR=$(readlink ${0}) || SCRIPT_LIBDIR_DIR=${0}
SCRIPT_LIBDIR_DIR="${SCRIPT_LIBDIR_DIR%/*}"


echo '# BEGIN SECTION: check variables'
if [ -z "${PULL_REQUEST_NUMBER}" ]; then
    echo PULL_REQUEST_NUMBER not specified
    exit -1
fi
echo '# END SECTION'

echo '# BEGIN SECTION: clean up environment'
bash -x ${SCRIPT_LIBDIR}/_homebrew_cleanup.bash
echo '# END SECTION'


echo '# BEGIN SECTION: run test-bot'
brew test-bot             \
    --tap=osrf/simulation \
    --bottle              \
    --ci-pr               \
    --verbose https://github.com/osrf/homebrew-simulation/pull/${PULL_REQUEST_NUMBER}
echo '# END SECTION'
