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

PULL_REQUEST_HEAD_REPO=git@github.com:osrfbuild/homebrew-simulation.git

. ${SCRIPT_LIBDIR}/_homebrew_github_setup.bash
. ${SCRIPT_LIBDIR}/_homebrew_github_get_formula_path.bash

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
# get line with formula version
VERSION_LINE=$(awk \
  "/^  version ['\"]/ {print FNR}" ${FORMULA_PATH} | head -1)
# check if version can be correctly auto-detected from url
if ${BREW} ruby -e "exit Version.parse(\"${SOURCE_TARBALL_URI}\").to_s == \"${VERSION}\""
then
  echo Version can be correctly auto-detected from URL
  if [ -n "${VERSION_LINE}" ]; then
    echo remove explicit version on line number ${VERSION_LINE}
    sed -i -e "${VERSION_LINE}d" ${FORMULA_PATH}
  fi
else
  if [ -z "${VERSION_LINE}" ]; then
    # Need to insert explicit version tag after url
    echo Adding explicit version tag after URL
    sed -i -e "${URI_LINE}a\  version \"${VERSION}\"" ${FORMULA_PATH}
  else
    echo Changing version to
    echo ${VERSION}
    echo on line number ${VERSION_LINE}
    sed -i -e "${VERSION_LINE}c\  version \"${VERSION}\"" ${FORMULA_PATH}
  fi
fi

echo
# get stable sha256 and its line number
SHA=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.checksum"`
echo Changing sha256 from
echo ${SHA} to
echo ${SOURCE_TARBALL_SHA}
SHA_LINE=`awk "/${SHA}/ {print FNR}" ${FORMULA_PATH} | head -1`
echo on line number ${SHA_LINE}
sed -i -e "${SHA_LINE}c\  sha256 \"${SOURCE_TARBALL_SHA}\"" ${FORMULA_PATH}

echo
# revision line if it's nonzero
FORMULA_REVISION=$(${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.pkg_version.revision")
if [ "$FORMULA_REVISION" -gt 0 ]; then
  echo Deleting formula revision $FORMULA_REVISION
  FORMULA_REVISION_LINE=$(awk "/  revision ${FORMULA_REVISION}/ {print FNR}" ${FORMULA_PATH} | head -1)
  echo on line number ${FORMULA_REVISION_LINE}
  sed -i -e "${FORMULA_REVISION_LINE}d" ${FORMULA_PATH}
fi

# create branch with name and sanitized version string
PULL_REQUEST_BRANCH="${PACKAGE_ALIAS}_`echo ${VERSION} | tr ' ~:^?*[' '_'`"

. ${SCRIPT_LIBDIR}/_homebrew_github_commit.bash
