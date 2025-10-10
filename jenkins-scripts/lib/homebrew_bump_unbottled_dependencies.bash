#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

echo '# BEGIN SECTION: check variables'
if [ -z "${FORMULA_WITH_UNBOTTLED_DEPENDENCIES}" ]; then
  echo FORMULA_WITH_UNBOTTLED_DEPENDENCIES not specified
  exit 1
fi
if [ -z "${BOTTLE_TAG}" ]; then
  echo BOTTLE_TAG not specified
  exit 1
fi
echo '# END SECTION'

PULL_REQUEST_HEAD_REPO=git@github.com:osrfbuild/homebrew-simulation.git

. ${SCRIPT_LIBDIR}/_homebrew_github_setup.bash

echo '# BEGIN SECTION: bump revisions of unbottled dependencies of specified formula'
for f in ${FORMULA_WITH_UNBOTTLED_DEPENDENCIES}
do
    for d in $($(brew --repo osrf/simulation)/.github/ci/unbottled_dependencies.sh $f ${BOTTLE_TAG})
    do
        echo Bumping revision for "$d" as it is an unbottled dependency of "$f"
        brew bump-revision --message="rebuild" "$d"
    done
done
echo '# END SECTION'

# create branch with name and sanitized version string
PULL_REQUEST_BRANCH="bump_unbottled_dependencies_$(date +%s)"
PULL_REQUEST_TITLE="${FORMULA_WITH_UNBOTTLED_DEPENDENCIES}: bump revision of unbottled dependencies"
SKIP_COMMIT=true

. ${SCRIPT_LIBDIR}/_homebrew_github_commit.bash
