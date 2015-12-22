#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

echo '# BEGIN SECTION: check variables'
if [ -z "${PACKAGE_ALIAS}" ]; then
  echo PACKAGE_ALIAS not specified
  exit -1
fi
if [ -z "${SOURCE_TARBALL_URI}" ]; then
  echo SOURCE_TARBALL_URI not specified
  exit -1
fi
if [ -z "${VERSION}" ]; then
  echo VERSION not specified
  exit -1
fi
if [ -z "${SOURCE_TARBALL_SHA}" ]; then
  echo SOURCE_TARBALL_SHA not specified, computing now
  echo
  SOURCE_TARBALL_SHA=`curl -L ${SOURCE_TARBALL_URI} \
    | shasum --algorithm 256 \
    | awk '{print $1}'`
fi
echo '# END SECTION'

. ${SCRIPT_LIBDIR}/_homebrew_github_setup.bash

echo '# BEGIN SECTION: calculating the SHA hash and changing the formula'
# get stable uri and its line number
URI=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.url"`
echo Changing url from
echo ${URI} to
echo ${SOURCE_TARBALL_URI}
URI_LINE=`grep -n ${URI} ${FORMULA_PATH} \
  | head -1 \
  | sed -e 's@:.*@@'`
echo on line number ${URI_LINE}
sed -i -e "${URI_LINE}c\  url \"${SOURCE_TARBALL_URI}\"" ${FORMULA_PATH}

echo
# get stable sha256 and its line number
SHA=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.checksum"`
echo Changing sha256 from
echo ${SHA} to
echo ${SOURCE_TARBALL_SHA}
SHA_LINE=`awk "/${SHA}/ {print FNR}" ${FORMULA_PATH} | head -1`
echo on line number ${SHA_LINE}
sed -i -e "${SHA_LINE}c\  sha256 \"${SOURCE_TARBALL_SHA}\"" ${FORMULA_PATH}

# create branch with name and sanitized version string
BRANCH="${PACKAGE_ALIAS}_`echo ${VERSION} | tr ' ~:^?*[' '_'`"

. ${SCRIPT_LIBDIR}/_homebrew_github_commit.bash
