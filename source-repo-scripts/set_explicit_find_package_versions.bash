#!/usr/bin/env bash
# Copyright (C) 2025 Open Source Robotics Foundation
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

# The script will set explicit find_package versions in each library of a given collection.
# Only the root CMakeLists.txt, python/CMakeLists.txt, and src/python_pybind11/CMakeLists.txt
# files will be modified.
#
# Requires the 'gh' CLI, `xmllint`, and 'python-vcstool' to be installed.
#
# Usage:
# $ ./set_explicit_find_package_versions.sh <collection> <issue_reference>
#
# For example, to set explicit versions in all find_package calls in Jetty packages:
#
# ./set_explicit_find_package_versions.sh jetty gazebosim/gz-jetty#138
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
ISSUE_REFERENCE=${2}

PR_BRANCH=find_explicit_version_${COLLECTION}
PR_TEXT="Part of ${ISSUE_REFERENCE}."

set -e

if [[ $# -lt 2 ]]; then
  echo "./set_explicit_find_package_versions.sh <collection> <issue_reference>"
  exit 1
fi

COMMIT_MSG="Find ${COLLECTION} packages with explicit version"
echo -e "${GREY}${WHITE_BG}${COMMIT_MSG}${DEFAULT_BG}${DEFAULT}"

TEMP_DIR="/tmp/set_explicit_find_package_versions"
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

  if git diff --exit-code; then
    echo -e "${GREEN}${REPO}: Nothing to commit for ${REPO}.${DEFAULT}"
    return
  fi

  # Sanity check that we're on a find_explicit_version branch already
  local CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  if [[ ! $CURRENT_BRANCH =~ find_explicit_version_* ]]
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
echo

# Loop over all packages and change find_package calls in specific CMakeLists.txt files
for pkg_xml in ${TEMP_DIR}/src/*/package.xml; do
  PACKAGE=$(xmllint --xpath '/package/name/text()' $pkg_xml)
  VERSION=$(xmllint --xpath '/package/version/text()' $pkg_xml)
  MAJOR_VERSION=$(echo $VERSION | sed -e 's@\..*@@')
  echo "Find ${PACKAGE} with explicit version ${MAJOR_VERSION}"
  pushd $(dirname $pkg_xml) > /dev/null
  for cmake_txt_path in CMakeLists.txt \
                        python/CMakeLists.txt \
                        src/python_pybind11/CMakeLists.txt
  do
    # For gz_find_package calls:
    # * If it has a VERSION argument on the first line that is followed by
    #   an argument containing digits and '.', then delete the following
    #   argument.
    #   For example, the following line:
    #       gz_find_package(gz-math REQUIRED VERSION 0.0.0)
    #   is replaced with
    #       gz_find_package(gz-math REQUIRED VERSION)
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@\(gz_find_package\s*(\s*${PACKAGE}.*\sVERSION\)\s\+[0-9\.]\+@\1@"
    # * If it has a VERSION argument on the first line, delete it
    #   For example, the following lines:
    #       gz_find_package(gz-math REQUIRED VERSION)
    #       gz_find_package(gz-math VERSION REQUIRED)
    #   are replaced with
    #       gz_find_package(gz-math REQUIRED)
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@\(gz_find_package\s*(\s*${PACKAGE}.*\)\sVERSION@\1@"
    # * Add "VERSION ${MAJOR_VERSION}" just after the ${PACKAGE} name to be found
    #   when the package name is at the end of line.
    #   For example, the following line:
    #       gz_find_package(gz-math
    #   is replaced with
    #       gz_find_package(gz-math VERSION 9
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@\(gz_find_package\s*(\s*${PACKAGE}\)\$@\1 VERSION ${MAJOR_VERSION}@"
    # * Add "VERSION ${MAJOR_VERSION}" just after the ${PACKAGE} name to be found
    #   when there is whitespace or ')' after the package name.
    #   For example, the following lines:
    #       gz_find_package(gz-math)
    #       gz_find_package(gz-math REQUIRED)
    #   are replaced with
    #       gz_find_package(gz-math VERSION 9)
    #       gz_find_package(gz-math VERSION 9 REQUIRED)
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@\(gz_find_package\s*(\s*${PACKAGE}\)\([ )]\)@\1 VERSION ${MAJOR_VERSION}\2@"

    # For find_package calls:
    # * If it has a string containing digits and '.' right after the package name
    #   to be found, then remove that string.
    #   For example, the following lines:
    #       find_package(gz-math 0.0.0)
    #       find_package(gz-math 0.0.0 REQUIRED)
    #   are replaced with
    #       find_package(gz-math)
    #       find_package(gz-math REQUIRED)
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@^\(\s*find_package\s*(\s*${PACKAGE}\)\s\+[0-9\.]\+@\1@"
    # * Add the major version after the package name to be found when the package
    #   name is at the end of line.
    #   For example, the following lines:
    #       find_package(gz-math
    #   is replaced with
    #       find_package(gz-math 9
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@^\(\s*find_package\s*(\s*${PACKAGE}\)\$@\1 ${MAJOR_VERSION}@"
    # * Add the major version after the package name to be found when there
    #   is whitespace or ')' after the package name.
    #   For example, the following lines:
    #       find_package(gz-math)
    #       find_package(gz-math REQUIRED)
    #   is replaced with
    #       find_package(gz-math 9)
    #       find_package(gz-math 9 REQUIRED)
    find ${TEMP_DIR}/src/*/${cmake_txt_path} -type f -print0 | xargs -0 \
        sed -i "s@^\(\s*find_package\s*(\s*${PACKAGE}\)\([ )]\)@\1 ${MAJOR_VERSION}\2@"
  done
  popd > /dev/null
done

# Create new branch, commit, and PR if there are changes
for pkg_xml in ${TEMP_DIR}/src/*/package.xml; do
  pushd $(dirname $pkg_xml)
  # Skip commitAndPR if repo has no code changes
  git diff --exit-code --quiet && continue

  BASE_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  git stash
  startFromCleanBranch ${PR_BRANCH} $BASE_BRANCH
  git stash pop
  commitAndPR ${GZ_ORG} ${BASE_BRANCH} ""
  popd
done
