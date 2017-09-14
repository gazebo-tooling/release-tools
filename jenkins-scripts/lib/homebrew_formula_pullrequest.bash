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
# check if formula has auto-detected version field
if ${BREW} ruby -e "exit \"${PACKAGE_ALIAS}\".f.stable.version.detected_from_url?"
then
  # check if auto-detected version is correct
  if ${BREW} ruby -e "exit \"${PACKAGE_ALIAS}\".f.stable.version.to_s == \"${VERSION}\""
  then
    echo Version is correctly auto-detected from URL
  else
    # Need to insert explicit version tag after url
    sed -i -e "${URI_LINE}a\  version \"${VERSION}\"" ${FORMULA_PATH}
  fi
else
  # get version and line number
  FORMULA_VERSION=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.version"`
  echo Changing version from
  echo ${FORMULA_VERSION} to
  echo ${VERSION}
  VERSION_LINE=`awk \
    "/version .${FORMULA_VERSION}/ {print FNR}" ${FORMULA_PATH} | head -1`
  echo on line number ${VERSION_LINE}
  sed -i -e "${VERSION_LINE}c\  version \"${VERSION}\"" ${FORMULA_PATH}
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
