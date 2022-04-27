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

# DESCRIPTION
# ===========
# This script will migrate headers and references to those headers in the source files for:
# - ign(ition) -> gz
# - Ign(ition) -> Gz
#
# It will, for all subdirectories under `src`, `test`, `examples`, and `include`:
# - Move the files
# - Edit source files to update references/new paths
# - Edit CMakeLists files to update references/new paths
#
# It will also leave redirection aliases for tick-tocking in the existing ign(ition) directories;
# As well as create a top level CMakeLists file to install those redirection aliases.
#
# WARNING
# =======
# This script will NOT migrate variable and class names! It also won't migrate macro names with
# arguments. (It'll mainly try to target header guards for macro updates.)
#
# PRE-REQUISITES
# ==============
# Requires the 'gh' CLI to be installed.
#
# USAGE
# =====
# Place the script in the root of the repo to be migrated, and run:
# $ ./header_migration_script.sh
#
# Author: methylDragon


# CONSTANTS ========================================================================================
export LICENSE="""\
/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the \"License\");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an \"AS IS\" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
"""

DEFAULT="\e[39m"
DEFAULT_BG="\e[49m"

GREY="\e[90m"
GREEN="\e[32m"
RED="\e[31m"
WHITE_BG="\e[107m"
BLUE_BG="\e[44m"
GREEN_BG="\e[42m"

IGN_ORG="ignitionrobotics"
DEFAULT_COMMIT_MSG="DEFAULT_COMMIT_MSG" # Bumps in ${COLLECTION}"
DEFAULT_PR_TITLE="DEFAULT_PR_TITLE"
DEFAULT_PR_TEXT="DEFAULT_PR_TEXT" # See https://github.com/${TOOLING_ORG}/release-tools/issues/${ISSUE_NUMBER}"


# GIT ==============================================================================================
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
    echo -e "${GREEN}${REPO}: Checking out branch ${PR_BRANCH}${DEFAULT}"
    git checkout $PR_BRANCH
    return
  fi

  # Check remote
  HAS_PR_BRANCH=$(git ls-remote --heads origin ${PR_BRANCH})
  if [[ ! -z ${HAS_PR_BRANCH} ]]; then
    echo -e "${GREEN}${REPO}: Checking out branch ${PR_BRANCH}${DEFAULT}"
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
# gitCommit [-a] [-c] [-f] [-s] ORG [COMMIT_MSG]
# Args:
# $1: Org name
# $2: Base branch to open PR against
# $3: Commit message
gitCommit() {
  local OPTIND ADD_ALL STAGED_DIFF FORCE_COMMIT
  while getopts acefs flag; do
    case "${flag}" in
      a)
        local ADD_ALL="true" && echo -e "${GREEN}[gitCommit] Adding all unstaged files to commit!${DEFAULT}"
        ;;
      c|s)
        local STAGED_DIFF="true" && echo -e "${GREEN}[gitCommit] Checking diff for staged files!${DEFAULT}"
        ;;
      e)
        local AMEND="true" && echo -e "${GREEN}[gitCommit] Amending commit!${DEFAULT}"
        ;;
      f)
        local FORCE_COMMIT="true" && echo -e "${GREEN}[gitCommit] Ignoring diffs!${DEFAULT}"
        ;;
      *)
        echo -e "${RED}[gitCommit] Invalid flag passed! Valid: -s -c -e -a ${DEFAULT}"
        ;;
      esac
  done
  shift $((OPTIND-1))

  local REPO=${PWD##*/}
  local ORG=$1
  local COMMIT_MSG=${2:-${DEFAULT_COMMIT_MSG}}

  if [[ "${FORCE_COMMIT}" == "true" ]] ; then
        echo -e "${GREEN}[gitCommit] Skipping diff check!${DEFAULT}"
  else
    if [[ "${STAGED_DIFF}" == "true" ]] ; then
      if git diff --staged --exit-code; then
        echo -e "${GREEN}${REPO}: Nothing to commit for ${REPO}.${DEFAULT}"
        return
      fi
    else
      if git diff --exit-code; then
        echo -e "${GREEN}${REPO}: Nothing to commit for ${REPO}.${DEFAULT}"
        return
      fi
    fi
  fi

  # Sanity check that we're on a right branch
  local CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  if [[ ! $CURRENT_BRANCH =~ header_migration.* ]] ; then
    echo -e "${RED}${REPO}: Something's wrong, trying to commit to branch ${CURRENT_BRANCH}.${DEFAULT}"
    return
  fi

  while true; do
    if [[ "${AMEND}" == "true" ]] ; then
      echo -e "${GREEN_BG}${REPO}: Amend latest commit on ${REPO}? (y/n)${DEFAULT_BG}"
    else
      echo -e "${GREEN_BG}${REPO}: Commit ${REPO}: ${COMMIT_MSG} ? (y/n)${DEFAULT_BG}"
    fi

    read CONTINUE

    case $CONTINUE in
      y) break;;
      n) echo -e "${GREEN}Skipping commit${DEFAULT}" && break;;
    esac
  done

  if [ "$CONTINUE" = "y" ]; then
    if [[ "${ADD_ALL}" == "true" ]] ; then
      git add -A
    fi
    if [[ "${AMEND}" == "true" ]] ; then
      git commit --amend -C HEAD
    else
      git commit -sam "${COMMIT_MSG}"
    fi
  fi
}


gitPushAndPR() {
  local REPO=${PWD##*/}
  local ORG=$1
  local BASE_BRANCH=$2
  local PR_TITLE=${3:-${DEFAULT_PR_TEXT}}
  local PR_TEXT=${4:-${DEFAULT_PR_TEXT}}

  # Sanity check that we're on a right branch
  local CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  if [[ ! $CURRENT_BRANCH =~ header_migration.* ]] ; then
    echo -e "${RED}${REPO}: Something's wrong, trying to commit to branch ${CURRENT_BRANCH}.${DEFAULT}"
    return
  fi

  echo -e "${GREEN_BG}${REPO}: Push to ${REPO} (${CURRENT_BRANCH}) and PR branch: ${CURRENT_BRANCH} -> ${BASE_BRANCH}? (y/n)${DEFAULT_BG}"
  read CONTINUE
  if [ "$CONTINUE" = "y" ]; then
    git push origin ${CURRENT_BRANCH}
    gh pr create --title "${PR_TITLE}" --body "${PR_TEXT}" --repo ${ORG}/${REPO} --base ${BASE_BRANCH}
  fi
}


# METHODS AND CONSTANTS ============================================================================
reviewConfirm() {
  while true; do
    echo -e "${GREEN_BG}Have you reviewed the changes above, and are you ready to commit? (y?)${DEFAULT_BG}"
    read CONTINUE

    case $CONTINUE in
      y) break;;
    esac
  done

  if [ "$CONTINUE" = "y" ]; then
    return
  fi
}

mvHeaders() {
  # Create necessary directories
  dirname $1 | sed 's@include\(.*\)*/ignition@include\1/gz@g' | xargs -I {} mkdir -pv {}

  # Move anything in an include/.../ignition or include/ignition subdirectory
  # Also handles:
  #   IgnitionXXX -> GzXXX
  #   IgnXXX -> GzXXX
  if [[ $1 =~ include(.*)*/ignition ]] || [[ $1 =~ include(.*)*/Ign(ition)?[A-Z] ]] ; then
    echo $1 | sed \
      -e 's@include\(.*\)*/ignition@include\1/gz@g' \
      -e 's@include\(.*\)*/Ign\(ition\)\?\([A-Z]\)@include\1/Gz\3@g' \
       | xargs -I {} bash -c "git mv -f $1 {} && cp {} $1 && echo '[MOVED WITH RESIDUAL] $1 --> {}'" _
  fi

  # Leave redirection aliases
  #   Converting *.in into normal versions (spoofing configuration of *.in files)
  if [[ $1 =~ .*\.in ]] ; then
    echo "[CONVERTING] $1"
    echo $1 | sed 's@\(.*\)\.in@\1@g' | xargs -I {} mv $1 {}
  fi

  # echo $1 | sed 's@\(.*\)\.in@\1@g' | xargs -I {} touch {}
} ; export -f mvHeaders


migrateSources() {  # Different variations of ignition/ign -> gz in source files
  # Note: DOES NOT MIGRATE CLASS OR VARIABLE NAMES

  if [[ $1 =~ \.c[^\.]*$|\.in[^\.]*$|\.h[^\.]*$ ]] ; then # Only do for source files
    echo "[MIGRATING SOURCES] $1"
    if [[ $1 != *.in ]] ; then                  # !! But not macros for .in files
      # Only migrate non-macro definition calls
      sed -i 's@\(#.*\) IGNITION_\([^(]*\)$@\1 GZ_\2@g' $1  # e.g. IGNITION_UTILS__XXX -> GZ_UTILS__XXX
      sed -i 's@\(#.*\) IGN_\([^(]*\)\$@\1 GZ_\2@g' $1       # e.g. IGN_UTILS__XXX -> GZ_UTILS__XXX
    fi

    # NOTE(CH3): We're not migrating class or variable names for now
    # sed -i 's@Ignition\([A-Z]\)@Gz\1@g' $1 # e.g. IgnitionFormatter -> GzFormatter

    sed -i 's@ignition/@gz/@g' $1   # e.g. include <ignition/utils/XXX> -> include <gz/utils/XXX>
    sed -i 's@ignition_@gz_@g' $1  # e.g. ignition_xxx -> gz_xxx

    # NOTE(CH3): The other one was too greedy (sign_bit -> sgz_bit)
    # sed -i 's@ign_@gz_@g' $1       # e.g. ign_xxx -> gz_xxx
    sed -i 's@\([{(_<"]\)ign_@\1gz_@g' $1             # e.g. ${ign_utils} -> ${gz_utils}

    # Rollback edge cases
    sed -i 's@${gz_headers}@${ign_headers}@g' $1
  fi
} ; export -f migrateSources


migrateCmake() {  # Different variations of ignition/ign -> gz in CMake files
  # This is special because we need to specifically avoid unmigrated ign-cmake macro calls

  if [[ $1 =~ CMakeLists.txt ]] ; then # Only do for CMakeLists files
    echo "[MIGRATING CMAKE] $1"

    sed -i 's@(ignition@(gz@g' $1   # e.g. add_subdirectory(ignition) -> add_subdirectory(gz)

    sed -i 's@Ignition\([A-Z]\)@Gz\1@g' $1  # e.g. IgnitionFormatter -> GzFormatter
    sed -i 's@(\(.*\)ignition@(\1gz@g' $1   # e.g. add_subdirectory(ignition) -> add_subdirectory(gz)

    # NOTE(CH3):
    # ^\([^ign ]\+\) ignores lines that start with ign(nition)_
    # Which should avoid changing most ign-cmake macro calls
    sed -i 's@\b\([^ign ]\+\)ignition/@\1gz/@g' $1  # e.g. include <ignition/utils/XXX> -> include <gz/utils/XXX>
    sed -i 's@\b\([^ign ]\+\)ignition_@\1gz_@g' $1  # e.g. ignition_xxx -> gz_xxx
    sed -i 's@\b\([^ign ]\+\)ign_@\1gz_@g' $1       # e.g. XXX_ign_xxx -> XXX_gz_xxx

    # /^#/! ignores lines starting with # (ignore comments)
    sed -i '/^#/!s@\(\w\) ign_@\1 gz_@g' $1              # e.g. XXX ign_xxx -> XXX gz_xxx

    # Catchall for open-parentheses/quotes
    sed -i 's@\([{(_<"]\)ign_@\1gz_@g' $1             # e.g. ${ign_utils} -> ${gz_utils}

    # Rollback edge cases
    # TODO(CH3): This becomes a helper once our project names are actually migrated
    #            We can remove the rollbacks then!
    sed -i 's@project(gz@project(ignition@g' $1
    sed -i 's@find_package(gz@find_package(ignition@g' $1
    sed -i 's@IGN_\([^_]*\)_VER ${gz@IGN_\1_VER ${ignition@g' $1
    sed -i 's@-Dgz@-Dignition@g' $1
  fi
} ; export -f migrateCmake


populateRedirectionAlias() {
  echo "[REDIRECTING] $1"
  echo "$LICENSE" > $1
  echo "#include <$(echo $1 | sed \
    -e 's@.*/include/@@g' \
    -e 's@ignition/@gz/@g' \
    -e 's@Ign\(ition\)\?@Gz@g')>" >> $1
} ; export -f populateRedirectionAlias

# MAIN =============================================================================================
startFromCleanBranch header_migration main

# Move headers
find . -regex '.*include\(.*\)*' -type f -print0 | xargs -0 -I {} bash -c 'mvHeaders {}' _

# Cleanup dangling files
find . -regex ".*/include.*/ignition.*/CMakeLists\.txt" -type f -print0 | xargs -0 -I {} rm {}  # Remove dangling .in config files
find . -regex ".*/include.*/ignition.*/.*\.in" -type f -print0 | xargs -0 -I {} rm {}  # Remove dangling ignition CMakeLists

reviewConfirm
gitCommit -s ${IGN_ORG} "Move header files with git mv"  # Commits staged git mv changes
echo -e "\n== Copying Redirection Aliases ==\n"
gitCommit -f -a ${IGN_ORG} "Create redirection aliases"  # Adds and commits unstaged redirections

# Create Export.hh and <lib>.hh
find . -regex './include.*/ignition/[^/]*' -type d -print0 \
  | xargs -0 -I {} bash -c 'touch "{}/Export.hh" \
                            && echo "[CREATED] {}/Export.hh"' _
find . -regex './include.*/ignition/[^/]*' -type d -print0 \
  | xargs -0 -I {} bash -c 'touch "$(sed "s@\(.*\)/\(.*\)\$@\1/\2.hh@g" <<< {})" \
                            && echo "[CREATED] $(sed "s@\(.*\)/\(.*\)\$@\1/\2.hh@g" <<< {})"'

# Provision redirection aliases
find . -regex '.*include.*/ignition.*\.h.*\|.*include\(.*\)*/Ign\(ition\)?[A-Z].*\.h.*' -type f -print0 \
  | xargs -0 -I {} bash -c 'populateRedirectionAlias {}' _

# Do some absolutely ridiculous awk sourcery to remove most instances of copyright edits
GIT_DIFF="$(git diff)"
PATCHED_GIT_DIFF=$(awk '{
  if(/^-.*Open Source Robotics Foundation/){
    prev=$0;
    getline;

    if(/^\+.*Open Source Robotics Foundation/){
      print prev; sub(/^-/, "+", prev); print prev;
    } else {
      print prev "\n" $0;
    }
  }
  else{print;}
}' <<< "${GIT_DIFF}")

git restore .
git apply <<< "${PATCHED_GIT_DIFF}"

reviewConfirm
gitCommit -f -a -e ${IGN_ORG}

# Migrate Gz sources
find . -regex '.*/\[src\|test\|examples\]/.*\|.*include\(.*\)*/[gz|Gz].*' -type f -print0 | xargs -0 -I {} bash -c 'migrateSources {}' _

reviewConfirm
gitCommit ${IGN_ORG} "Migrate sources in src, test, examples, and include"

# Migrate Gz CMake files
find . -regex '.*[include\|src\|test\|examples].*/CMakeLists\.txt' -type f -print0 | xargs -0 -I {} bash -c 'migrateCmake {}' _

# Add header level CMakeLists.txt
touch ./include/CMakeLists.txt
echo "add_subdirectory(gz)" > ./include/CMakeLists.txt
echo "install(DIRECTORY ignition DESTINATION \${IGN_INCLUDE_INSTALL_DIR_FULL})" >> ./include/CMakeLists.txt

# Edit top level CMakeLists
sed -i 's@ign_configure_project(\(.*\))@ign_configure_project(\n  REPLACE_IGNITION_INCLUDE_PATH gz/utils\n  \1)@g' CMakeLists.txt

reviewConfirm
gitCommit ${IGN_ORG} "Migrate CMake files"
gitPushAndPR ${IGN_ORG} main "ign -> gz Header Migration" "Test run of https://github.com/ignition-tooling/release-tools/pull/712"

# TODOs
# Parse libs from CLI
# Clone
