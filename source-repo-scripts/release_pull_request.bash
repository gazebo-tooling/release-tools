#!/usr/bin/env bash
# Copyright (C) 2022 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# The script will open a pull request a release PR (actually opens the PR page in your web browser)
#
# Requires the 'gh' CLI to be installed.
#
# Usage:
# $ ./release_pull_request.bash <from_branch> <to_branch>
#
# For example, to release `gz-rendering7` 7.1.0
#
# ./release_pull_request.bash 7.1.0 gz-rendering7
#
# Make sure you've checked out the branch that has the changes for the release
# and that the changes have been pushed.

VERSION=${1}
TO_BRANCH=${2}

if [[ $# -ne 2 ]]; then
  echo "./release_pull_request.bash <version> <to_branch>"
  exit 1
fi

set -e

git fetch --tags
PREV_VER=$(git describe --tags --abbrev=0 | sed 's/.*_//')

LOCAL_BRANCH=$(git rev-parse --abbrev-ref  HEAD)
REMOTE_BRANCH=$(git rev-parse --abbrev-ref  HEAD@{upstream})
REMOTE=${REMOTE_BRANCH/\/$LOCAL_BRANCH/}
CURRENT_BRANCH="${REMOTE}:${LOCAL_BRANCH}"

ORIGIN_URL=$(git remote get-url origin)
ORIGIN_ORG_REPO=$(echo ${ORIGIN_URL} | sed -e 's@.*github\.com.@@' | sed -e 's/\.git//g')

if [[ $PREV_VER == $VERSION ]] then
  echo "Previous version ($PREV_VER) and current ($VERSION) version should be different"
  exit 1
fi

TITLE="Prepare for ${VERSION} Release"

BODY="# 🎈 Release

Preparation for ${VERSION} release.

Comparison to ${PREV_VER}: https://github.com/${ORIGIN_ORG_REPO}/compare/${PREV_TAG}...${TO_BRANCH}

<!-- Add links to PRs that require this release (if needed) -->
Needed by <PR(s)>

## Checklist
- [ ] Asked team if this is a good time for a release
- [ ] There are no changes to be ported from the previous major version
- [ ] No PRs targeted at this major version are close to getting in
- [ ] Bumped minor for new features, patch for bug fixes
- [ ] Updated changelog
- [ ] Updated migration guide (as needed)
- [ ] Link to PR updating dependency versions in appropriate repository in [gazebo-release](https://github.com/gazebo-release) (as needed): <LINK>

<!-- Please refer to https://github.com/gazebo-tooling/release-tools#for-each-release for more information -->

**Note to maintainers**: Remember to use **Squash-Merge** and edit the commit message to match the pull request summary while retaining \`Signed-off-by\` messages."


gh pr create \
    --title "$TITLE" \
    --repo "$ORIGIN_ORG_REPO" \
    --base "$TO_BRANCH" \
    --body "$BODY" \
    --head "$CURRENT_BRANCH" \
    --web
