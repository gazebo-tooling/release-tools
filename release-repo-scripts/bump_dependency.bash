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
# $ ./bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number> <docs_branch>
#
# For example, to bump to `ign-rendering6` and all its dependencies, as well as
# `sdf12` and its dependencies on fortress:
#
# ./bump_dependency.bash fortress "ign-rendering;sdformat" "6;12" 428 "chapulina/fortress"
#
# The script clones all the necessary repositories under /tmp/bump_dependency.
#
# Before committing to each repository, the script asks "Commit <repository name>?".
# Before saying yes, navigate to the repository and check if the diff looks reasonable.
# When you say yes, the changes will be committed and pushed. Click on the link printed
# by GitHub to open the pull request.

# TODO: Update gzdev to use nightlies

DEFAULT="\e[39m"
DEFAULT_BG="\e[49m"

GREY="\e[90m"
GREEN="\e[32m"
RED="\e[31m"
WHITE_BG="\e[107m"
BLUE_BG="\e[44m"
GREEN_BG="\e[42m"

#IGN_ORG="ignitionrobotics"
#OSRF_ORG="osrf"
#TOOLING_ORG="ignition-tooling"
#RELEASE_ORG="ignition-release"
IGN_ORG="chapulina"
OSRF_ORG="chapulina"
TOOLING_ORG="chapulina"
RELEASE_ORG="chapulina"

COLLECTION=${1}
LIBRARY_INPUT=${2}
VERSION_INPUT=${3}
# Number of release-tools issue to link back to
ISSUE_NUMBER=${4}
DOCS_BRANCH=${5}

COMMIT_MSG="Bumps in ${COLLECTION}"
PR_TEXT="See https://github.com/${TOOLING_ORG}/release-tools/issues/${ISSUE_NUMBER}"

set -e

if [[ $# -lt 3 ]]; then
  echo "./bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number>"
  exit 1
fi

echo -e "${GREY}${WHITE_BG}Bumps in ${COLLECTION}${DEFAULT_BG}${DEFAULT}"

IFS=';'
read -a LIBRARIES <<< "${LIBRARY_INPUT}"
read -a VERSIONS <<< "${VERSION_INPUT}"

TEMP_DIR="/tmp/bump_dependency"

echo -e "${GREEN}Creating directory [${TEMP_DIR}]${DEFAULT}"

mkdir -p ${TEMP_DIR}

# gazebodistro

cd ${TEMP_DIR}
if [ ! -d "gazebodistro" ]; then
  echo -e "${GREEN}Cloning gazebodistro${DEFAULT}"
  git clone https://github.com/${TOOLING_ORG}/gazebodistro
else
  echo -e "${GREEN}gazebodistro is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/gazebodistro
git fetch
git reset --hard
git checkout master
git pull

# docs

cd ${TEMP_DIR}
if [ ! -d "docs" ]; then
  echo -e "${GREEN}Cloning docs${DEFAULT}"
  git clone https://github.com/${IGN_ORG}/docs
else
  echo -e "${GREEN}docs is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/docs
git fetch
git checkout ${DOCS_BRANCH}
git pull
git reset --hard

# homebrew

cd ${TEMP_DIR}
if [ ! -d "homebrew-simulation" ]; then
  echo -e "${GREEN}Cloning homebrew-simulation${DEFAULT}"
  git clone https://github.com/${OSRF_ORG}/homebrew-simulation
else
  echo -e "${GREEN}homebrew-simulation is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/homebrew-simulation
git fetch
git checkout master
git pull
git reset --hard

# release-tools

cd ${TEMP_DIR}
if [ ! -d "release-tools" ]; then
  echo -e "${GREEN}Cloning release-tools${DEFAULT}"
  git clone https://github.com/${TOOLING_ORG}/release-tools
else
  echo -e "${GREEN}release-tools is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/release-tools
git fetch
git checkout master
git pull
git reset --hard

# This first loop finds out what downstream dependencies also need to be bumped
for ((i = 0; i < "${#LIBRARIES[@]}"; i++)); do

  LIB=${LIBRARIES[$i]}
  VER=${VERSIONS[$i]}
  PREV_VER="$((${VER}-1))"

  ##################
  # gazebodistro
  ##################
  cd ${TEMP_DIR}/gazebodistro

  YAML_FILE=${LIB}${VER}.yaml
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

  # Assume all files that have some `main` or `master` branch may be bumped
  PREV_RELEASE_BRANCH=${LIB}${PREV_VER}
  PREV_RELEASE_BRANCH="${PREV_RELEASE_BRANCH/sdformat/sdf}"
  grep -rl "main\|master" *.yaml | xargs sed -i "s ${PREV_RELEASE_BRANCH} ${MAIN_BRANCH} g"

  # Add all bumped dependencies to the list to be bumped
  DIFF=$(git diff --name-only)

  while IFS= read -r TO_BUMP; do

    TO_BUMP=${TO_BUMP%.yaml}
    if [[ "$TO_BUMP" =~ ^([a-z-]+)([0-9]+) ]]; then

      if [[ ${#BASH_REMATCH[*]} -lt 3 ]]; then
        continue
      fi

      BUMP_LIB=${BASH_REMATCH[1]}
      BUMP_VER=${BASH_REMATCH[2]}

      if [[ ! " ${LIBRARIES[@]} " =~ " ${BUMP_LIB} " ]]; then
        echo -e "${GREEN}Also bumping ${BUMP_LIB}${BUMP_VER}${DEFAULT}"
        LIBRARIES+=($BUMP_LIB)
        VERSIONS+=($BUMP_VER)
      fi
    fi

  done <<< "$DIFF"

  sed -i "s ${LIB}${PREV_VER} main g" collection-${COLLECTION}.yaml

  ##################
  # docs
  ##################
  cd ${TEMP_DIR}/docs
  sed -i -E "s ((${LIB}.*))${PREV_VER} \1${VER} g" ${COLLECTION}/install.md

  # TODO: populate gzdev here too

done

# Clean up changes in gazebodistro, we'll be redoing them below one file at a time
cd ${TEMP_DIR}/gazebodistro
git reset --hard

# Add collection to libraries, without version
LIBRARIES+=(ign-$COLLECTION)

##################
# docs
##################
cd ${TEMP_DIR}/docs
git diff
echo -e "${GREEN_BG}Commit docs and open PR? (y/n)${DEFAULT_BG}"
read CONTINUE
if [ "$CONTINUE" = "y" ] ; then
  git checkout -b bump_${COLLECTION}
  git commit -sam"${COMMIT_MSG}"
  git push origin bump_${COLLECTION}
  gh pr create --title "${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${IGN_ORG}/docs --base ${DOCS_BRANCH}
fi

# TODO: commit gzdev

for ((i = 0; i < "${#LIBRARIES[@]}"; i++)); do

  LIB=${LIBRARIES[$i]}
  VER=${VERSIONS[$i]}
  PREV_VER="$((${VER}-1))"
  LIB_UPPER=`echo ${LIB#"ign-"} | tr a-z A-Z`
  ORG=${IGN_ORG}
  if [ "$LIB" = "sdformat" ]; then
    ORG=${OSRF_ORG}
  fi
  BUMP_BRANCH="bump_${COLLECTION}_${LIB}${VER}"

  echo -e "${BLUE_BG}Processing [${LIB}]${DEFAULT_BG}"

  ##################
  # source code
  ##################

  echo -e "${GREEN}${LIB}: source code${DEFAULT}"

  cd ${TEMP_DIR}
  if [ ! -d "${LIB}" ]; then
    echo -e "${GREEN}Cloning ${LIB}${DEFAULT}"
    git clone https://github.com/${ORG}/${LIB}
  else
    echo -e "${GREEN}${LIB} is already cloned${DEFAULT}"
  fi
  cd ${TEMP_DIR}/${LIB}

  git fetch
  git reset --hard

  HAS_BUMP=$(git ls-remote --heads origin ${BUMP_BRANCH})
  if [[ ! -z ${HAS_BUMP} ]]; then
    echo -e "${GREEN}Checking out ${LIB} branch [$BUMP_BRANCH]${DEFAULT}"
    git checkout $BUMP_BRANCH
  else
    echo -e "${GREEN}Checking out ${LIB} branch [main]${DEFAULT}"
    git checkout main
    git pull
  fi

  # Check if main branch of that library is the correct version
  PROJECT_NAME="${LIB//-/_}${VER}"
  PROJECT_NAME="${PROJECT_NAME/ign_/ignition-}"
  PROJECT="project.*(${PROJECT_NAME}"
  if ! grep -q ${PROJECT} "CMakeLists.txt"; then
    echo -e "${RED}Wrong project name on [CMakeLists.txt], looking for [$PROJECT_NAME].${DEFAULT}"
    exit
  fi

  echo -e "${GREEN}Updating source code${DEFAULT}"
  # TODO: handle yaml file in collection source like gazebodistro (main instead of dep+1)
  for ((j = 0; j < "${#LIBRARIES[@]}"; j++)); do

    DEP_LIB=${LIBRARIES[$j]#"ign-"}
    DEP_VER=${VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    find . -type f ! -name 'Changelog.md' ! -name 'Migration.md' -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"

    # Second run with _ instead of -, to support multiple variations of fuel-tools
    DEP_LIB=${DEP_LIB//-/_}
    find . -type f ! -name 'Changelog.md' ! -name 'Migration.md' -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"
  done

  if ! git diff --exit-code; then
    echo -e "${GREEN_BG}Commit ${LIB} and open PR? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      if [[ -z ${HAS_BUMP} ]]; then
        git checkout -b $BUMP_BRANCH
      fi
      git commit -sam"${COMMIT_MSG}"
      git push origin ${BUMP_BRANCH}
      gh pr create --title "${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${ORG}/${LIB} --base main
    fi
  else
    echo -e "${GREEN}Nothing to commit for ${LIB}.${DEFAULT}"
  fi
  SOURCE_COMMIT=`git rev-parse HEAD`

  TODO: Continue from here

  ##################
  # release repo
  ##################

  echo -e "${GREEN}${LIB}: release repo${DEFAULT}"

  cd ${TEMP_DIR}
  RELEASE_REPO=${LIB}${VER}-release
  if [ ! -d "${RELEASE_REPO}" ]; then
    echo -e "${GREEN}Cloning ${RELEASE_REPO}${DEFAULT}"
    git clone https://github.com/${RELEASE_ORG}/${RELEASE_REPO}
  else
    echo -e "${GREEN}${RELEASE_REPO} is already cloned${DEFAULT}"
  fi
  cd ${TEMP_DIR}/${RELEASE_REPO}

  git fetch
  HAS_MAIN=$(git ls-remote --heads origin main)
  if [[ -z ${HAS_MAIN} ]]; then
    git checkout master
  else
    git checkout main
  fi
  git pull
  git reset --hard

  echo -e "${GREEN}Updating release repository${DEFAULT}"
  for ((j = 0; j < "${#LIBRARIES[@]}"; j++)); do

    DEP_LIB=${LIBRARIES[$j]#"ign-"}
    DEP_VER=${VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    find . -type f -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"
  done

  if ! git diff --exit-code; then
    echo -e "${GREEN_BG}Commit ${RELEASE_REPO} and open PR? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      git checkout -b ${BUMP_BRANCH}
      git commit -sam"${COMMIT_MSG}"
      git push origin ${BUMP_BRANCH}
      gh pr create --title "${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${RELEASE_ORG}/${RELEASE_REPO} --base main
    fi
  else
    echo -e "${GREEN}Nothing to commit for ${RELEASE_REPO}.${DEFAULT}"
  fi

  ##################
  # homebrew
  ##################

  echo -e "${GREEN}${LIB}: homebrew${DEFAULT}"

  cd ${TEMP_DIR}/homebrew-simulation

  git fetch
  git reset --hard
  HAS_MAIN=$(git ls-remote --heads origin main)
  BUMP_BRANCH="bump_${COLLECTION}_${LIB}"
  HAS_BUMP=$(git ls-remote --heads origin ${BUMP_BRANCH})
  if [[ ! -z ${HAS_BUMP} ]]; then
    echo -e "${GREEN}Checking out homebrew-simulation branch [$BUMP_BRANCH]${DEFAULT}"
    git checkout $BUMP_BRANCH
  elif [[ ! -z ${HAS_MAIN} ]]; then
    echo -e "${GREEN}Checking out homebrew-simulation branch [main]${DEFAULT}"
    git checkout main
  else
    echo -e "${GREEN}Checking out homebrew-simulation branch [master]${DEFAULT}"
    git checkout master
  fi

  FORMULA="Formula/${LIB/ign/ignition}${VER}.rb"
  if [ ! -f "$FORMULA" ]; then
    echo -e "${GREEN}Creating ${FORMULA}${DEFAULT}"

    git rm Aliases/${LIB/ign/ignition}${VER}
    cp Formula/${LIB/ign/ignition}${PREV_VER}.rb $FORMULA
    git add $FORMULA
  fi

  echo -e "${GREEN}Updating ${FORMULA}${DEFAULT}"
  URL="https://github.com/${ORG}/${LIB}/archive/${SOURCE_COMMIT}.tar.gz"
  wget $URL
  SHA=`sha256sum ${SOURCE_COMMIT}.tar.gz | cut -d " " -f 1`
  rm ${SOURCE_COMMIT}.tar.gz

  # ign-libN -> main
  sed -i "s ${LIB}${PREV_VER} main g" $FORMULA
  # libN
  sed -i -E "s ((${LIB#"ign-"}))${PREV_VER} \1${VER} g" $FORMULA
  # class IgnitionLibN
  sed -i -E "s/((class Ignition.*))${PREV_VER}/\1${VER}/g" $FORMULA
  # remove bottle - TODO: this is only needed for new formulae
  sed -i -e "/bottle do/,/end/d" $FORMULA
  # URL from release to commit - TODO: remove manual step
  sed -i "s@url.*@url \"$URL\"@g" $FORMULA
  # SHA - TODO: remove manual step
  sed -i "s/sha256.*/sha256 \"$SHA\"/g" $FORMULA
  # version - TODO: fix duplicating existing one
  sed -i "/url.*/a \ \ version\ \"${PREV_VER}.999.999~0~`date +"%Y%m%d"`~${SOURCE_COMMIT:0:6}\"" $FORMULA
  # Remove extra blank lines
  cat -s $FORMULA | tee $FORMULA

  for ((j = 0; j < "${#LIBRARIES[@]}"; j++)); do

    DEP_LIB=${LIBRARIES[$j]#"ign-"}
    DEP_VER=${VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g" $FORMULA
  done

  if ! git diff --exit-code; then
    echo -e "${GREEN_BG}Clean up the files before committing!${DEFAULT_BG}"
    echo -e "${GREEN_BG}Commit homebrew-simulation for ${LIB} and open PR? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      if [[ -z ${HAS_BUMP} ]]; then
        git checkout -b $BUMP_BRANCH
      fi
      git commit -sam"${COMMIT_MSG}"
      git push origin $BUMP_BRANCH
      gh pr create --title "${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${OSRF_ORG}/homebrew-simulation --base master
    fi
  else
    echo -e "${GREEN}Nothing to commit for homebrew-simulation.${DEFAULT}"
  fi

  # Collection ends here
  if ! [[ $VER == ?(-)+([0-9]) ]] ; then
    continue
  fi

  ##################
  # release-tools
  ##################
  cd ${TEMP_DIR}/release-tools

  # TODO: build nightlies from main

#  DOCKER_SCRIPT="jenkins-scripts/docker/${LIB/ign-/ign_}-compilation.bash"
#  echo -e "${GREEN}Updating [${DOCKER_SCRIPT}]${DEFAULT}"
#
#  for ((j = 0; j < "${#LIBRARIES[@]}"; j++)); do
#
#    DEP_UPPER=`echo ${LIBRARIES[$j]#"ign-"} | tr a-z A-Z`
#    DEP_VER=${VERSIONS[$j]}
#
#    if [[ $LIB_UPPER == $DEP_UPPER ]] ; then
#      continue
#    fi
#
#    if ! [[ $DEP_VER == ?(-)+([0-9]) ]] ; then
#      continue
#    fi
#
#    sed -i "/.*GZDEV.*/i \
#if\ [[\ \$\{IGN_${LIB_UPPER}_MAJOR_VERSION}\ -eq\ ${VER}\ ]];\ then\n\
#  export\ BUILD_IGN_${DEP_UPPER}=true\n\
#  export\ IGN_${DEP_UPPER}_MAJOR_VERSION=${DEP_VER}\n\
#  export\ IGN_${DEP_UPPER}_BRANCH=main\n\
#fi"\
#    $DOCKER_SCRIPT
#  done

  echo -e "${GREEN_BG}Commit release-tools? (y/n)${DEFAULT_BG}"
  read CONTINUE
  if [ "$CONTINUE" = "y" ] ; then
    cd ${TEMP_DIR}/release-tools
    git checkout -b ${BUMP_BRANCH}
    git commit -sam"${COMMIT_MSG}"
    git push origin $BUMP_BRANCH
    gh pr create --title "${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${TOOLING_ORG}/release-tools --base master
  fi

  ##################
  # gazebodistro
  ##################

  echo -e "${GREEN_BG}Commit gazebodistro for ${LIB} and open PR? (y/n)${DEFAULT_BG}"
  read CONTINUE
  if [ "$CONTINUE" = "y" ] ; then
    cd ${TEMP_DIR}/gazebodistro
    git checkout -b ${BUMP_BRANCH}
    git commit -sam"${COMMIT_MSG}"
    git push origin $BUMP_BRANCH
    gh pr create --title "${COMMIT_MSG}" --body "${PR_TEXT}" --repo ${TOOLING_ORG}/gazebodistro --base master
  fi

done




