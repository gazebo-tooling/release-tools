#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

echo '# BEGIN SECTION: check variables'
if [ -z "${BROKEN_FORMULA}" ]; then
  echo BROKEN_FORMULA not specified
  exit -1
fi
if [ -z "${BOTTLE_TAG}" ]; then
  echo BOTTLE_TAG not specified
  exit -1
fi
echo '# END SECTION'

PULL_REQUEST_HEAD_REPO=git@github.com:osrfbuild/homebrew-simulation.git

. ${SCRIPT_LIBDIR}/_homebrew_github_setup.bash

echo '# BEGIN SECTION: remove bottles of broken formula and dependents'
for f in ${BROKEN_FORMULA}
do
    brew bump-revision --remove-bottle-block --message="broken bottle" $f
    for d in $($(brew --repo osrf/simulation)/.github/ci/bottled_dependents.sh $f ${BOTTLE_TAG})
    do
        brew bump-revision --remove-bottle-block --message="broken bottle" $d
    done
done
echo '# END SECTION'

# create branch with name and sanitized version string
PULL_REQUEST_BRANCH="remove_broken_bottles_$(date +%s)"
PULL_REQUEST_TITLE="Remove bottles for ${BROKEN_FORMULA} and dependents"
SKIP_COMMIT=1

. ${SCRIPT_LIBDIR}/_homebrew_github_commit.bash
