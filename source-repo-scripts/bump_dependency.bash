#!/usr/bin/env bash
# Copyright (C) 2020 Open Source Robotics Foundation
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

# The script will bump the major version of a library for a given collection
#
# Requires the 'gh' CLI to be installed.
#
# Usage:
# $ ./bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number> <prev_collection> [<docs_branch>]
#
# For example, to bump to `gz-rendering6` and all its dependencies, as well as
# `sdf12` and its dependencies on fortress using the `chapulina/fortress` branch for `docs`:
#
# ./bump_dependency.bash fortress "gz-rendering;sdformat" "6;12" 428 edifice "chapulina/fortress"
#
# The `docs_branch` parameter is optional and defaults to `master` if not specified.
#
# The script clones all the necessary repositories under /tmp/bump_dependency.
#
# Before committing to each repository, the script asks "Commit <repository name>?".
# Before saying yes, navigate to the repository and check if the diff looks reasonable.
# When you say yes, the changes will be committed and pushed. Click on the link printed
# by GitHub to open the pull request.

# TODO: Update gz-collection.yaml on release-tools

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
LIBRARY_INPUT=${2}
VERSION_INPUT=${3}
# Number of release-tools issue to link back to
ISSUE_NUMBER=${4}
PREV_COLLECTION=${5}
DOCS_BRANCH=${6}
if [[ -z "${DOCS_BRANCH}" ]]; then
  DOCS_BRANCH=master
fi

COMMIT_MSG="Bumps in ${COLLECTION}"
PR_TEXT="See https://github.com/${TOOLING_ORG}/release-tools/issues/${ISSUE_NUMBER}"

set -e

if [[ $# -lt 5 ]]; then
  echo "./bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number>"
  exit 1
fi

TEMP_DIR="/tmp/bump_dependency"
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

  if git diff --exit-code; then
    echo -e "${GREEN}${REPO}: Nothing to commit for ${REPO}.${DEFAULT}"
    return
  fi

  # Sanity check that we're on a bump branch already
  local CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  if [[ ! $CURRENT_BRANCH =~ bump_* ]]
  then
    echo -e "${RED}${REPO}: Something's wrong, trying to commit to branch ${CURRENT_BRANCH}.${DEFAULT}"
    return
  fi

  local LIB=${CURRENT_BRANCH#bump_${COLLECTION}}
  local LIB=${LIB#_}
  local LIB=": ${LIB}"

  echo -e "${GREEN_BG}${REPO}: Commit ${REPO} and open PR? (y/n)${DEFAULT_BG}"
  read CONTINUE
  if [ "$CONTINUE" = "y" ]; then
    git commit -sam"${COMMIT_MSG} ${LIB}"
    git push origin ${CURRENT_BRANCH}
    gh pr create --title "${COMMIT_MSG} ${LIB}" --body "${PR_TEXT}" --repo ${ORG}/${REPO} --base ${BASE_BRANCH}
  fi
}

echo -e "${GREY}${WHITE_BG}Bumps in ${COLLECTION}${DEFAULT_BG}${DEFAULT}"

IFS=';'
read -a LIBRARIES <<< "${LIBRARY_INPUT}"
read -a VERSIONS <<< "${VERSION_INPUT}"

# gazebodistro
cloneIfNeeded ${TOOLING_ORG} gazebodistro
startFromCleanBranch bump_${COLLECTION} master

# docs
cloneIfNeeded ${GZ_ORG} docs
startFromCleanBranch bump_${COLLECTION} ${DOCS_BRANCH}

# homebrew
cloneIfNeeded ${OSRF_ORG} homebrew-simulation
startFromCleanBranch bump_${COLLECTION} master

# release-tools
cloneIfNeeded ${TOOLING_ORG} release-tools
startFromCleanBranch bump_${COLLECTION} master

# This first loop finds out what downstream dependencies also need to be updated
# Store library name with major version in UNSORTED_PACKAGES
# Start with empty array
read -a UNSORTED_PACKAGES <<< ""
for ((i = 0; i < "${#LIBRARIES[@]}"; i++)); do

  LIB=${LIBRARIES[$i]}
  VER=${VERSIONS[$i]}
  PREV_VER="$((${VER}-1))"
  LIBVER=${LIB}${VER}

  # Append this library to be bumped to UNSORTED_PACKAGES
  UNSORTED_PACKAGES+=(${LIBVER})

  ##################
  # gazebodistro
  ##################
  cd ${TEMP_DIR}/gazebodistro

  YAML_FILE=${LIBVER}.yaml
  echo -e "${BLUE_BG}Processing [${YAML_FILE}]${DEFAULT_BG}"

  if [ ! -f "${YAML_FILE}" ]; then
    # TODO: Create it copying from version-1
    echo -e "${RED}${YAML_FILE} does not exist.${DEFAULT}"
    exit
  fi

  MAIN_BRANCH=main
  if ! grep -q $MAIN_BRANCH "${YAML_FILE}"; then
    MAIN_BRANCH=master
    if ! grep -q $MAIN_BRANCH "${YAML_FILE}"; then
      echo -e "${RED}No main or master branch found on ${YAML_FILE}.${DEFAULT}"
      exit
    fi
  fi

  PREV_RELEASE_BRANCH=${LIB}${PREV_VER}
  PREV_RELEASE_BRANCH="${PREV_RELEASE_BRANCH/sdformat/sdf}"
  # For any files containing `main` or `master`,
  # Replace every instance of `PREV_RELEASE_BRANCH` with `main`
  grep -rl "main\|master" *.yaml | \
    xargs sed -i "s ${PREV_RELEASE_BRANCH} ${MAIN_BRANCH} g"

  # Add the names of any modified files to the LIBRARIES and VERSIONS arrays
  # to ensure those packages receive further processing
  MODIFIED_FILES=$(git diff --name-only)

  while IFS= read -r TO_UPDATE; do

    TO_UPDATE=${TO_UPDATE%.yaml}
    if [[ "$TO_UPDATE" =~ ^([a-z-]+)([0-9]+) ]]; then

      if [[ ${#BASH_REMATCH[*]} -lt 3 ]]; then
        continue
      fi

      UPDATED_LIB=${BASH_REMATCH[1]}
      UPDATED_VER=${BASH_REMATCH[2]}

      if [[ ! " ${LIBRARIES[@]} " =~ " ${UPDATED_LIB} " ]]; then
        echo -e "${GREEN}Also updating ${UPDATED_LIB}${UPDATED_VER}${DEFAULT}"
        LIBRARIES+=($UPDATED_LIB)
        VERSIONS+=($UPDATED_VER)
        UNSORTED_PACKAGES+=(${TO_UPDATE})
      fi
    fi

  done <<< "$MODIFIED_FILES"

  sed -i "s ${PREV_RELEASE_BRANCH} main g" collection-${COLLECTION}.yaml

  ##################
  # docs
  ##################
  cd ${TEMP_DIR}/docs
  sed -i -E "s ((${LIB}.*))${PREV_VER} \1${VER} g" ${COLLECTION}/install.md

done

# Sort gazebodistro yaml files in topological order
cd ${TEMP_DIR}/gazebodistro
SORTED_YAML=$(
  for p in ${UNSORTED_PACKAGES[@]}; do
    # Reorder UNSORTED_PACKAGES array in order of gazebodistro yaml file size
    # This is a heuristic for topological order
    wc -l $p.yaml
  done \
  | sort -n \
  | uniq \
  | awk '{ print $2 }'
)

# Converted file names in SORTED_YAML to matched arrays of library names and version numbers
echo -e "${GREEN}Sorting in topological order the packages to update:${DEFAULT}"
read -a SORTED_LIBRARIES <<< ""
read -a SORTED_VERSIONS <<< ""
while IFS= read -r package; do
  LIB=$(basename -s .yaml ${package} | sed -e 's@[0-9]*$@@')
  VER=$(basename -s .yaml ${package} | sed -e 's@^[^0-9]*@@')
  SORTED_LIBRARIES+=($LIB)
  SORTED_VERSIONS+=($VER)
  echo -e "${GREEN}$LIB version $VER${DEFAULT}"
done <<< ${SORTED_YAML}

# Clean up changes in gazebodistro, we'll be redoing them below one file at a time
cd ${TEMP_DIR}/gazebodistro
git reset --hard

# Add collection to libraries, without version
SORTED_LIBRARIES+=(gz-$COLLECTION)

##################
# docs
##################
cd ${TEMP_DIR}/docs
commitAndPR ${GZ_ORG} ${DOCS_BRANCH}

for ((i = 0; i < "${#SORTED_LIBRARIES[@]}"; i++)); do

  LIB=${SORTED_LIBRARIES[$i]}

  LIB_=${LIB//-/_} # For fuel_tools
  VER=${SORTED_VERSIONS[$i]}
  PREV_VER="$((${VER}-1))"
  LIB_UPPER=$(echo ${LIB#"gz-"} | tr a-z A-Z)
  ORG=${GZ_ORG}
  BUMP_BRANCH="ci_matching_branch/bump_${COLLECTION}_${LIB}${VER}"

  echo -e "${BLUE_BG}Processing [${LIB}]${DEFAULT_BG}"

  ##################
  # release repo
  ##################

  echo -e "${GREEN}${LIB}: release repo${DEFAULT}"

  RELEASE_REPO=${LIB}${VER}-release
  cloneIfNeeded ${RELEASE_ORG} ${RELEASE_REPO}
  startFromCleanBranch ${BUMP_BRANCH} main

  for ((j = 0; j < "${#SORTED_LIBRARIES[@]}"; j++)); do

    DEP_LIB=${SORTED_LIBRARIES[$j]#"gz-"}
    DEP_VER=${SORTED_VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    find . -type f -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"
  done

  commitAndPR ${RELEASE_ORG} main

  ##################
  # homebrew
  ##################

  echo -e "${GREEN}${LIB}: homebrew${DEFAULT}"

  cd ${TEMP_DIR}/homebrew-simulation
  startFromCleanBranch ${BUMP_BRANCH} master

  # construct path with major version suffix
  FORMULA_BASE=$LIB
  FORMULA="Formula/${FORMULA_BASE}${VER}.rb"
  if [ ! -f "$FORMULA" ]; then
    echo -e "${GREEN}${LIB}: Creating ${FORMULA}${DEFAULT}"

    git rm Aliases/${FORMULA_BASE}${VER}

    # Collection
    if ! [[ $VER == ?(-)+([0-9]) ]] ; then
      cp Formula/gz-${PREV_COLLECTION}.rb $FORMULA
    else
      cp Formula/${FORMULA_BASE}${PREV_VER}.rb $FORMULA
    fi

    git add $FORMULA
  fi

  echo -e "${GREEN}${LIB}: Updating ${FORMULA}${DEFAULT}"
  URL="https://github.com/${ORG}/${LIB}.git"

  # libN
  sed -i -E "s ((${LIB#"gz-"}))${PREV_VER} \1${VER} g" $FORMULA
  sed -i -E "s ((${LIB_#"gz_"}))${PREV_VER} \1${VER} g" $FORMULA
  # lib-N
  sed -i -E "s ((${LIB#"gz-"}-))${PREV_VER} \1${VER} g" $FORMULA
  sed -i -E "s ((${LIB_#"gz_"}-))${PREV_VER} \1${VER} g" $FORMULA
  # gz-libN -> main
  sed -i "s ${LIB}${PREV_VER} main g" $FORMULA
  # class GzLibN
  sed -i -E "s/((class Gz.*))${PREV_VER}/\1${VER}/g" $FORMULA
  sed -i -E "s/((class Sdformat))${PREV_VER}/\1${VER}/g" $FORMULA
  # remove bottle - TODO: this is only needed for new formulae
  sed -i -e "/bottle do/,/end/d" $FORMULA
  # URL from release to commit - TODO: remove manual step
  sed -i "s@^  url.*@  url \"$URL\", branch: \"main\"@g" $FORMULA
  # SHA - remove in favor of building from `main` branch
  sed -i "/^  sha256.*/d" $FORMULA
  # revision - remove if present
  sed -i "/^  revision.*/d" $FORMULA
  # version
  PREV_VER_NONNEGATIVE=$([[ "${PREV_VER}" -lt 0 ]] && echo "0" || echo "${PREV_VER}")
  sed -i "/ version /d" $FORMULA
  sed -i "/^  url.*/a\  version \"${PREV_VER_NONNEGATIVE}.999.999-0-$(date +"%Y%m%d")\"" $FORMULA
  # Collection
  if [[ "${LIB}" == "gz-${COLLECTION}" ]]; then
    PREV_COLLECTION_CAPITALIZED="${PREV_COLLECTION^}"
    COLLECTION_CAPITALIZED="${COLLECTION^}"
    sed -i -E "s/((Gz))${PREV_COLLECTION_CAPITALIZED}/\1${COLLECTION_CAPITALIZED}/g" $FORMULA
    sed -i -E "s/((gz-))${PREV_COLLECTION}/\1${COLLECTION}/g" $FORMULA
  fi
  # Remove extra blank lines
  cat -s $FORMULA | tee $FORMULA

  for ((j = 0; j < "${#SORTED_LIBRARIES[@]}"; j++)); do

    DEP_LIB=${SORTED_LIBRARIES[$j]#"gz-"}

    DEP_VER=${SORTED_VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g" $FORMULA
  done

  commitAndPR ${OSRF_ORG} master

  ##################
  # gazebodistro
  ##################
  echo -e "${GREEN}${LIB}: gazebodistro${DEFAULT}"

  cd ${TEMP_DIR}/gazebodistro
  startFromCleanBranch ${BUMP_BRANCH} master

  # Collection
  if ! [[ $VER == ?(-)+([0-9]) ]] ; then
    YAML_FILE=collection-${COLLECTION}.yaml
  else
    YAML_FILE=${LIB}${VER}.yaml
  fi
  for ((j = 0; j < "${#SORTED_LIBRARIES[@]}"; j++)); do

    DEP_LIB=${SORTED_LIBRARIES[$j]/sdformat/sdf}
    DEP_VER=${SORTED_VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    sed -i "s ${DEP_LIB}${DEP_PREV_VER} main g" $YAML_FILE
    DEP_LIB=${DEP_LIB//-/_} # fuel_tools
    sed -i "s ${DEP_LIB}${DEP_PREV_VER} main g" $YAML_FILE
  done

  commitAndPR ${TOOLING_ORG} master

  ##################
  # source code
  ##################

  echo -e "${GREEN}${LIB}: source code${DEFAULT}"

  cloneIfNeeded ${ORG} ${LIB}
  startFromCleanBranch ${BUMP_BRANCH} main

  # Check if main branch of that library is the correct version
  PROJECT_NAME="${LIB}${VER}"
  PROJECT="project.*(${PROJECT_NAME}"
  echo -e "${GREEN}  checking versioned project name ${PROJECT_NAME}${DEFAULT}"
  if ! grep -q ${PROJECT} "CMakeLists.txt"; then
    PROJECT_NAME="${LIB}"
    PROJECT="project.*(${PROJECT_NAME}[^0-9]"
    echo -e "${GREEN}  checking unversioned project name ${PROJECT_NAME}${DEFAULT}"
    if ! grep -q ${PROJECT} "CMakeLists.txt"; then
      echo -e "${RED}Wrong project name on [CMakeLists.txt], looking for [$PROJECT_NAME].${DEFAULT}"
      exit
    fi
  fi

  echo -e "${GREEN}${LIB}: Updating source code${DEFAULT}"
  for ((j = 0; j < "${#SORTED_LIBRARIES[@]}"; j++)); do

    DEP_LIB=${SORTED_LIBRARIES[$j]#"gz-"}

    DEP_VER=${SORTED_VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    # Replace lines like "find_package(gz-cmake2 2.0.0)"
    #               with "find_package(gz-cmake3)"
    find . -type f -name 'CMakeLists.txt' -print0 | xargs -0 sed -i "s@\(find_package.*${DEP_LIB}\)${DEP_PREV_VER} \+${DEP_PREV_VER}[^ )]*@\1${DEP_VER}@g"

    # Replace lines like "gz_find_package(gz-math6 VERSION 6.5.0)"
    #               with "gz_find_package(gz-math7)"
    # Preserves other args and handles edge cases:
    #               like "gz_find_package(gz-math6 VERSION 6.5.0 REQUIRED)"
    #               with "gz_find_package(gz-math6 REQUIRED)"
    #               like "gz_find_package(gz-math6 REQUIRED COMPONENTS VERSION 6.10 eigen3)"
    #               with "gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)"
    find . -type f -name 'CMakeLists.txt' -print0 | xargs -0 sed -i "s@\(find_package.*${DEP_LIB}\)${DEP_PREV_VER}\(.*\) \+VERSION \+${DEP_PREV_VER}[^ )]*@\1${DEP_VER}\2@g"


    # Rule: *plugin2 -> *plugin3
    # Replace lines like: "find_package(gz-cmake2)"
    #               with: "find_package(gz-cmake3)"
    find . -type f ! -name 'Changelog.md' ! -name 'Migration.md' -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"

    # Second run with _ instead of -, to support multiple variations of fuel-tools
    DEP_LIB=${DEP_LIB//-/_}
    find . -type f ! -name 'Changelog.md' ! -name 'Migration.md' -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"
  done

  commitAndPR ${ORG} main

  # Collection ends here
  if ! [[ $VER == ?(-)+([0-9]) ]] ; then
    continue
  fi

  ##################
  # release-tools
  ##################
  echo -e "${GREEN}${LIB}: release-tools${DEFAULT}"

  cd ${TEMP_DIR}/release-tools
  startFromCleanBranch ${BUMP_BRANCH} master

  # Build nightlies from main
  LIB_SHORT=${LIB/sdformat/sdf}
  DSL_FILE="jenkins-scripts/dsl/ignition_collection.dsl"
  sed -i "s/\(debbuild.*\)${LIB}${PREV_VER}\(.*\)${LIB_SHORT}${PREV_VER}/\1${LIB}${VER}\2main/g" $DSL_FILE

  commitAndPR ${TOOLING_ORG} master

done
