#!/bin/bash -x
set -e

[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

get_formula()
{
  local filename=${1}

  echo ${filename/-*}
}

get_osX_distribution()
{
  local hash_line=${1}

  echo ${hash_line/*:}
}

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
# return always true since audit fails to run gzserver
# can not find a way of disabling it
( brew test-bot           \
    --tap=osrf/simulation \
    --bottle              \
    --ci-pr               \
    --verbose ${PULL_REQUEST_URL} ) || true
echo '# END SECTION'

echo '# BEGIN SECTION: export bottle'
if [[ $(find . -name *.bottle.* | wc -l) -lt 2 ]]; then
 echo "Can not find the two bottle files"
 exit -1
fi

mv *.bottle.tar.gz ${PKG_DIR}
mv *.bottle.rb ${PKG_DIR}

FILENAME=$(ls *.bottle.rb)
FORMULA=$(get_formula ${FILENAME})
NEW_HASH_LINE=$(grep 'sha256.*=>' $FILENAME)
DISTRO=$(get_osX_distribution ${NEW_HASH_LINE})

echo "PACKAGE_ALIAS=${FORMULA}" > exported_vars.properties
echo "NEW_HASH_LINE=${NEW_HASH_LINE}" >> exported_vars.properties
echo "DISTRO=${DISTRO}" >> exported_vars.properties

echo '# END SECTION'
