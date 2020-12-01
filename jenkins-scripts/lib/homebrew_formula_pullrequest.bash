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

echo '# BEGIN SECTION: checking if version needs to be passed'
if ${BREW} ruby -e "exit Version.parse(\"${SOURCE_TARBALL_URI}\").to_s == \"${VERSION}\""
then
  echo Version can be correctly auto-detected from URL
else
  echo Adding explicit version tag
  EXPLICIT_VERSION="--version=${VERSION}"
fi
echo '# END SECTION'

echo '# BEGIN SECTION: call brew bump-formula-pr --dry-run'
${BREW} bump-formula-pr ${PACKAGE_ALIAS} --dry-run \
    --url="${SOURCE_TARBALL_URI}" --sha256="${SOURCE_TARBALL_SHA}" ${EXPLICIT_VERSION}
echo '# END SECTION'
echo '# BEGIN SECTION: call brew bump-formula-pr --write'
${BREW} bump-formula-pr ${PACKAGE_ALIAS} --write \
    --url="${SOURCE_TARBALL_URI}" --sha256="${SOURCE_TARBALL_SHA}" ${EXPLICIT_VERSION}
echo '# END SECTION'

# create branch with name and sanitized version string
PULL_REQUEST_BRANCH="${PACKAGE_ALIAS}_`echo ${VERSION} | tr ' ~:^?*[' '_'`"

. ${SCRIPT_LIBDIR}/_homebrew_github_commit.bash
