#!/usr/bin/env bash
# Copyright (C) 2026 Open Source Robotics Foundation
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

# The script will copy and rename an existing Ubuntu metadata folder in
# the release repositories of a given collection to add support for a new
# Ubuntu release.
#
# Requires the 'gh' CLI, `xmllint`, and 'python-vcstool' to be installed.
#
# Usage:
# $ ./collection_copy_ubuntu_metadata.bash <collection> <existing_ubuntu> <new_ubuntu> <issue_reference>
#
# For example, to copy the noble folder to resolute in all Jetty release repositories:
#
# ./collection_copy_ubuntu_metadata.bash jetty noble resolute gazebo-tooling/release-tools/issues/1485
#
# Before committing to each repository, the script asks "Commit <repository name>?".
# Before saying yes, navigate to the repository and check if the diff looks reasonable.
# When you say yes, the changes will be committed and pushed. Click on the link printed
# by GitHub to open the pull request.

# default to false
DRY_RUN=${DRY_RUN:-false}

DEFAULT="\e[39m"
DEFAULT_BG="\e[49m"

GREY="\e[90m"
GREEN="\e[32m"
RED="\e[31m"
WHITE_BG="\e[107m"
BLUE_BG="\e[44m"
GREEN_BG="\e[42m"

GZ_ORG="gazebosim"
OSRF_ORG="osrf"
TOOLING_ORG="gazebo-tooling"
RELEASE_ORG="gazebo-release"

COLLECTION=${1}
OLD_DISTRO=${2}
NEW_DISTRO=${3}
ISSUE_REFERENCE=${4}

PR_BRANCH=${COLLECTION}_copy_${OLD_DISTRO}_to_${NEW_DISTRO}
PR_TEXT="Part of ${ISSUE_REFERENCE}."

set -e

if [[ $# -lt 4 ]]; then
  echo "./collection_copy_ubuntu_metadata.bash <collection> <existing_ubuntu> <new_ubuntu> <issue_reference>"
  exit 1
fi

COMMIT_MSG="${COLLECTION}: copy ${OLD_DISTRO} metadata to ${NEW_DISTRO}"
echo -e "${GREY}${WHITE_BG}${COMMIT_MSG}${DEFAULT_BG}${DEFAULT}"

TEMP_DIR="/tmp/collection_copy_ubuntu_metadata"
echo -e "${GREEN}Creating directory [${TEMP_DIR}]${DEFAULT}"
mkdir -p ${TEMP_DIR}

# Return the passed branch if provided, or main / master
getBaseBranch() {

  local BASE_BRANCH=$1

  # If no base branch provided, use main or master
  if [ -z "$BASE_BRANCH" ]; then
    HAS_MAIN=$(git ls-remote --heads origin main)
    if [[ ! -z ${HAS_MAIN} ]]; then
      local BASE_BRANCH="main"
    else
      local BASE_BRANCH="master"
    fi
  fi

  echo "$BASE_BRANCH"
}

# Clone repository into temp dir if not cloned yet and move to that folder
# Args:
# $1: Organization
# $2: Repository
cloneIfNeeded() {

  cd ${TEMP_DIR}

  local ORG=$1
  local REPO=$2

  if [ ! -d "$REPO" ]; then
    echo -e "${GREEN}${REPO}: Cloning ${ORG}/${REPO}${DEFAULT}"
    git clone https://github.com/${ORG}/${REPO}
  else
    echo -e "${GREEN}${REPO}: ${REPO} is already cloned${DEFAULT}"
  fi

  cd $REPO
}

# Helper to checkout a clean branch
# Args:
# $1: Branch to open PR from
# $2: Branch to build on top of
startFromCleanBranch() {

  local PR_BRANCH=$1
  local BASE_BRANCH=$2
  local REPO=${PWD##*/}

  git fetch
  git reset --hard

  # If PR branch exists, checkout and start fresh

  # Check local
  HAS_PR_BRANCH=$(git branch --list ${PR_BRANCH})
  if [[ ! -z ${HAS_PR_BRANCH} ]]; then
    echo -e "${GREEN}${REPO}: Checkout out branch ${PR_BRANCH}${DEFAULT}"
    git checkout $PR_BRANCH
    return
  fi

  # Check remote
  HAS_PR_BRANCH=$(git ls-remote --heads origin ${PR_BRANCH})
  if [[ ! -z ${HAS_PR_BRANCH} ]]; then
    echo -e "${GREEN}${REPO}: Checkout out branch ${PR_BRANCH}${DEFAULT}"
    git checkout $PR_BRANCH
    git pull
    return
  fi

  local BASE_BRANCH=$(getBaseBranch $BASE_BRANCH)

  # Make sure base branch exists
  HAS_BASE_BRANCH=$(git ls-remote --heads origin ${BASE_BRANCH})
  if [[ -z ${HAS_BASE_BRANCH} ]]; then
    echo -e "${RED}${REPO}: Branch ${BASE_BRANCH} does not exist.${DEFAULT}"
    return
  fi

  # Create PR branch off base
  echo -e "${GREEN}${REPO}: Checking out ${BASE_BRANCH}${DEFAULT}"
  git checkout $BASE_BRANCH
  git pull
  echo -e "${GREEN}${REPO}: Creating new branch ${PR_BRANCH}${DEFAULT}"
  git checkout -b $PR_BRANCH
}

# Commit and open PR
# Args:
# $1: Org name
# $2: Base branch to open PR against
commitAndPR() {
  local REPO=${PWD##*/}
  local ORG=$1
  local BASE_BRANCH=$2
  local COMMIT_MSG_PREFIX=$3

  if git diff --exit-code && git diff --cached --exit-code; then
    echo -e "${GREEN}${REPO}: Nothing to commit for ${REPO}.${DEFAULT}"
    return
  fi

  # Sanity check that we're on a find_explicit_version branch already
  local CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  if [[ ! $CURRENT_BRANCH =~ [a-z]*_copy_[a-z]*_to_[a-z]* ]]
  then
    echo -e "${RED}${REPO}: Something's wrong, trying to commit to branch ${CURRENT_BRANCH}.${DEFAULT}"
    return
  fi

  if ${DRY_RUN}; then
    echo -e "${GREEN_BG}${REPO}: dry-run enabled (avoid commit and PR). Press to continue${DEFAULT_BG}"
    read CONTINUE
  else
    echo -e "${GREEN_BG}${REPO}: Commit ${REPO} and open PR? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      git commit -sam"${COMMIT_MSG_PREFIX}${COMMIT_MSG}"
      git push origin ${CURRENT_BRANCH}
      gh pr create --title "${COMMIT_MSG_PREFIX}${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${ORG}/${REPO} --base ${BASE_BRANCH}
    fi
  fi
}

# Clone gazebodistro to get the collection yaml file
cloneIfNeeded ${TOOLING_ORG} gazebodistro
COLLECTION_YAML_FILE=collection-${COLLECTION}.yaml
echo -e "${BLUE_BG}Importing [${COLLECTION_YAML_FILE}]${DEFAULT_BG}"

if [ ! -f "${COLLECTION_YAML_FILE}" ]; then
  echo -e "${RED}${COLLECTION_YAML_FILE} does not exist.${DEFAULT}"
  exit
fi

# Use vcstool to import the collection repositories
mkdir -p ${TEMP_DIR}/src
vcs import ${TEMP_DIR}/src < ${COLLECTION_YAML_FILE}
pushd ${TEMP_DIR}/src
git clone https://github.com/${GZ_ORG}/gz-${COLLECTION}
popd
echo

# Loop over all packages, clone and modify the corresponding release repositories
for pkg_xml in ${TEMP_DIR}/src/*/package.xml; do
  PACKAGE=$(xmllint --xpath '/package/name/text()' $pkg_xml)
  if [ "$PACKAGE" = "gz-$COLLECTION" ]; then
    RELEASE_REPO=gz-$COLLECTION-release
  elif [ "$COLLECTION" = "rotary" ]; then
    # remove "gz-" prefix and translate '_' to '-'
    DESIGNATION=$(echo ${PACKAGE#gz-} | tr '_' '-')
    RELEASE_REPO=gz-rotary-$DESIGNATION-release
  else
    VERSION=$(xmllint --xpath '/package/version/text()' $pkg_xml)
    MAJOR_VERSION=$(echo $VERSION | sed -e 's@\..*@@')
    RELEASE_REPO=$PACKAGE$MAJOR_VERSION-release
  fi
  echo "Clone release repo $RELEASE_REPO"
  cloneIfNeeded ${RELEASE_ORG} ${RELEASE_REPO}
  mkdir -p $NEW_DISTRO
  cp -R $OLD_DISTRO/* $NEW_DISTRO/
  echo Replace "$OLD_DISTRO" with "$NEW_DISTRO" in changelog
  sed -i -e "s@$OLD_DISTRO@$NEW_DISTRO@" $NEW_DISTRO/debian/changelog
  git add $NEW_DISTRO

  # Skip commitAndPR if repo has no staged or unstaged code changes
  git diff --cached --exit-code --quiet && git diff --exit-code --quiet && continue

  BASE_BRANCH=main
  git stash
  startFromCleanBranch ${PR_BRANCH} $BASE_BRANCH
  git stash pop
  commitAndPR ${RELEASE_ORG} ${BASE_BRANCH} ""
done
