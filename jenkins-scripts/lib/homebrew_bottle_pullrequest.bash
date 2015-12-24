#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

set +e

echo '# BEGIN SECTION: check variables'
if [ -z "${PACKAGE_ALIAS}" ]; then
  echo PACKAGE_ALIAS not specified
  exit -1
fi

if [ -z "${DISTRO}" ]; then
  echo DISTRO not specified
  exit -1
fi

if [ -z "${VERSION}" ]; then
  echo VERSION not specified
  exit -1
fi

if [ -z "${NEW_HASH_LINE}" ]; then
  echo NEW_HASH_LINE not specified
  exit -1
fi
echo '# END SECTION'

. ${SCRIPT_DIR}/lib/_homebrew_github_setup.bash

sed -i -e "s/sha256.*=>.*${DISTRO}/${NEW_HASH_LINE}/g" ${FORMULA_PATH}

. ${SCRIPT_DIR}/lib/_homebrew_github_commit.bash
