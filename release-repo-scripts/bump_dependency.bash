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
# Use:
# $ ./bump_dependency.bash <collection> <library> <version>

# TODO: Update gzdev to use nightlies

DEFAULT="\e[39m"
DEFAULT_BG="\e[49m"

GREY="\e[90m"
GREEN="\e[32m"
RED="\e[31m"
WHITE_BG="\e[107m"
BLUE_BG="\e[44m"
GREEN_BG="\e[42m"

COLLECTION=${1}
LIBRARY=${2}
VERSION=${3}

set -e

if [[ $# -lt 1 ]]; then
  echo "bump_dependency.bash <collection> <library> <version>"
  exit 1
fi

echo -e "${GREY}${WHITE_BG}Bump in ${COLLECTION}: ${LIBRARY}${VERSION}${DEFAULT_BG}${DEFAULT}"

LIBRARIES=(${LIBRARY})
VERSIONS=(${VERSION})

TEMP_DIR="/tmp/bump_dependency"

echo -e "${GREEN}Creating directory [${TEMP_DIR}]${DEFAULT}"

mkdir -p ${TEMP_DIR}

# gazebodistro

cd ${TEMP_DIR}
if [ ! -d "gazebodistro" ]; then
  echo -e "${GREEN}Cloning gazebodistro${DEFAULT}"
  git clone https://github.com/ignition-tooling/gazebodistro
else
  echo -e "${GREEN}gazebodistro is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/gazebodistro
git fetch
# TODO revert to master
git checkout d944dfd406d9d54e89d5a0aaf5806d125357eb92
#git pull
git reset --hard

# docs

cd ${TEMP_DIR}
if [ ! -d "docs" ]; then
  echo -e "${GREEN}Cloning docs${DEFAULT}"
  git clone https://github.com/ignitionrobotics/docs
else
  echo -e "${GREEN}docs is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/docs
git fetch
HAS_MAIN=$(git ls-remote --heads origin main)
if [[ -z ${HAS_MAIN} ]]; then
  git checkout master
else
  git checkout main
fi
git pull
git reset --hard

# homebrew

cd ${TEMP_DIR}
if [ ! -d "homebrew-simulation" ]; then
  echo -e "${GREEN}Cloning homebrew-simulation${DEFAULT}"
  git clone https://github.com/osrf/homebrew-simulation
else
  echo -e "${GREEN}homebrew-simulation is already cloned${DEFAULT}"
fi

# release-tools

cd ${TEMP_DIR}
if [ ! -d "release-tools" ]; then
  echo -e "${GREEN}Cloning release-tools${DEFAULT}"
  git clone https://github.com/ignition-tooling/release-tools
else
  echo -e "${GREEN}release-tools is already cloned${DEFAULT}"
fi

cd ${TEMP_DIR}/release-tools
git fetch
git checkout master
git pull
git reset --hard

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

  if ! grep -q main "${YAML_FILE}"; then
    echo -e "${RED}No main branch found on ${YAML_FILE}.${DEFAULT}"
    exit
  fi

  # Assume all files that have some `main` branch may be bumped
  grep -rl "main" *.yaml | xargs sed -i "s ${LIB}${PREV_VER} main g"

  # Add all bumped dependencies to the list to be bumped
  DIFF=$(git diff --name-only)
  for TO_BUMP in ${DIFF[@]}; do

    TO_BUMP=${TO_BUMP%.yaml}
    BUMP_LIB=${TO_BUMP::-1}
    BUMP_VER=${TO_BUMP: -1}

    if [[ ! " ${LIBRARIES[@]} " =~ " ${BUMP_LIB} " ]]; then
      echo -e "${GREEN}Also bumping ${BUMP_LIB}${BUMP_VER}${DEFAULT}"
      LIBRARIES+=($BUMP_LIB)
      VERSIONS+=($BUMP_VER)
    fi

  done

  sed -i "s ${LIB}${PREV_VER} main g" collection-${COLLECTION}.yaml

  ##################
  # docs
  ##################
  cd ${TEMP_DIR}/docs
  sed -i -E "s ((${LIB}.*))${PREV_VER} \1${VER} g" ${COLLECTION}/install.md

done

echo -e "${GREEN_BG}Commit gazebodistro? (y/n)${DEFAULT_BG}"
read CONTINUE
if [ "$CONTINUE" = "y" ] ; then
  cd ${TEMP_DIR}/gazebodistro
  git checkout -b bump_${COLLECTION}_${LIBRARY}
  git commit -sam"Bump in ${COLLECTION}: ${LIBRARY}${VERSION}"
  git push
fi

echo -e "${GREEN_BG}Commit docs? (y/n)${DEFAULT_BG}"
read CONTINUE
if [ "$CONTINUE" = "y" ] ; then
  cd ${TEMP_DIR}/docs
  git checkout -b bump_${COLLECTION}_${LIBRARY}
  git commit -sam"Bump in ${COLLECTION}: ${LIBRARY}${VERSION}"
  git push
fi

for ((i = 0; i < "${#LIBRARIES[@]}"; i++)); do

  LIB=${LIBRARIES[$i]}
  VER=${VERSIONS[$i]}
  PREV_VER="$((${VER}-1))"
  LIB_UPPER=`echo ${LIB#"ign-"} | tr a-z A-Z`

  if ! [[ $VER == ?(-)+([0-9]) ]] ; then
    continue
  fi

  echo -e "${BLUE_BG}Processing [${LIB}]${DEFAULT_BG}"

  ##################
  # source code
  ##################

  echo -e "${GREEN}${LIB}: source code${DEFAULT}"

  cd ${TEMP_DIR}
  if [ ! -d "${LIB}" ]; then
    echo -e "${GREEN}Cloning ${LIB}${DEFAULT}"
    git clone https://github.com/ignitionrobotics/${LIB}
  else
    echo -e "${GREEN}${LIB} is already cloned${DEFAULT}"
  fi
  cd ${TEMP_DIR}/${LIB}

  git fetch
  git reset --hard

  BUMP_BRANCH="bump_${COLLECTION}_${LIBRARY}"
  HAS_BUMP=$(git ls-remote --heads origin ${BUMP_BRANCH})
  if [[ ! -z ${HAS_BUMP} ]]; then
    echo -e "${GREEN}Checking out ${LIB} branch [$BUMP_BRANCH]${DEFAULT}"
    git checkout $BUMP_BRANCH
  else
    echo -e "${GREEN}Checking out ${LIB} branch [main]${DEFAULT}"
    git checkout main
  fi
  git pull

  # Check if main branch of that library is the correct version
  PROJECT="project(${LIB/ign/ignition}${VER}"
  if ! grep -q ${PROJECT} "CMakeLists.txt"; then
    echo -e "${RED}Wrong project name on [CMakeLists.txt], looking for [$PROJECT].${DEFAULT}"
    exit
  fi

  echo -e "${GREEN}Updating source code${DEFAULT}"
  for ((j = 0; j < "${#LIBRARIES[@]}"; j++)); do

    DEP_LIB=${LIBRARIES[$j]#"ign-"}
    DEP_VER=${VERSIONS[$j]}
    DEP_PREV_VER="$((${DEP_VER}-1))"

    find . -type f ! -name 'Changelog.md' ! -name 'Migration.md' -print0 | xargs -0 sed -i "s ${DEP_LIB}${DEP_PREV_VER} ${DEP_LIB}${DEP_VER} g"
  done

  if ! git diff --exit-code; then
    echo -e "${GREEN_BG}Commit ${LIB}? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      if [[ ! -z ${HAS_BUMP} ]]; then
        git checkout -b $BUMP_BRANCH
      fi
      git commit -sam"Bump in ${COLLECTION}: ${LIBRARY}${VERSION}"
      git push
    fi
  else
    echo -e "${GREEN}Nothing to commit for ${LIB}.${DEFAULT}"
  fi
  SOURCE_COMMIT=`git rev-parse HEAD`

  ##################
  # release repo
  ##################

  echo -e "${GREEN}${LIB}: release repo${DEFAULT}"

  cd ${TEMP_DIR}
  RELEASE_REPO=${LIB}${VER}-release
  if [ ! -d "${RELEASE_REPO}" ]; then
    echo -e "${GREEN}Cloning ${RELEASE_REPO}${DEFAULT}"
    git clone https://github.com/ignition-release/${RELEASE_REPO}
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
    echo -e "${GREEN_BG}Commit ${RELEASE_REPO}? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      git checkout -b bump_${COLLECTION}_${LIBRARY}
      git commit -sam"Bump in ${COLLECTION}: ${LIBRARY}${VERSION}"
      git push
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
  URL="https://github.com/ignitionrobotics/${LIB}/archive/${SOURCE_COMMIT}.tar.gz"
  wget $URL
  SHA=`sha256sum ${SOURCE_COMMIT}.tar.gz | cut -d " " -f 1`
  rm ${SOURCE_COMMIT}.tar.gz

  # ign-libN -> main
  sed -i "s ${LIB}${PREV_VER} main g" $FORMULA
  # libN
  sed -i -E "s ((${LIB#"ign-"}))${PREV_VER} \1${VER} g" $FORMULA
  # class IgnitionLibN
  sed -i -E "s/((class Ignition.*))${PREV_VER}/\1${VER}/g" $FORMULA
  # remove bottle
  sed -i -e "/bottle do/,/end/d" $FORMULA
  # URL from release to commit - TODO: remove manual step
  sed -i "s@url.*@url \"$URL\"@g" $FORMULA
  # SHA - TODO: remove manual step
  sed -i "s/sha256.*/sha256 \"$SHA\"/g" $FORMULA
  # version
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
    echo -e "${GREEN_BG}Commit homebrew-simulation for ${LIB}? (y/n)${DEFAULT_BG}"
    read CONTINUE
    if [ "$CONTINUE" = "y" ]; then
      if [[ -z ${HAS_BUMP} ]]; then
        git checkout -b $BUMP_BRANCH
      fi
      git commit -sam"Bump in ${COLLECTION}: ${LIB}${VERSION}"
      git push
    fi
  else
    echo -e "${GREEN}Nothing to commit for homebrew-simulation.${DEFAULT}"
  fi

  ##################
  # release-tools
  ##################
  cd ${TEMP_DIR}/release-tools

  DOCKER_SCRIPT="jenkins-scripts/docker/${LIB/ign-/ign_}-compilation.bash"
  echo -e "${GREEN}Updating [${DOCKER_SCRIPT}]${DEFAULT}"

  for ((j = 0; j < "${#LIBRARIES[@]}"; j++)); do

    DEP_UPPER=`echo ${LIBRARIES[$j]#"ign-"} | tr a-z A-Z`
    DEP_VER=${VERSIONS[$j]}

    if [[ $LIB_UPPER == $DEP_UPPER ]] ; then
      continue
    fi

    if ! [[ $DEP_VER == ?(-)+([0-9]) ]] ; then
      continue
    fi

    sed -i "/.*GZDEV.*/i \
if\ [[\ \$\{IGN_${LIB_UPPER}_MAJOR_VERSION}\ -eq\ ${VER}\ ]];\ then\n\
  export\ BUILD_IGN_${DEP_UPPER}=true\n\
  export\ IGN_${DEP_UPPER}_MAJOR_VERSION=${DEP_VER}\n\
  export\ IGN_${DEP_UPPER}_BRANCH=main\n\
fi"\
    $DOCKER_SCRIPT
  done

done

echo -e "${GREEN_BG}Clean up the files before committing!${DEFAULT_BG}"
echo -e "${GREEN_BG}Commit release-tools? (y/n)${DEFAULT_BG}"
read CONTINUE
if [ "$CONTINUE" = "y" ] ; then
  cd ${TEMP_DIR}/release-tools
  git checkout -b bump_${COLLECTION}_${LIBRARY}
  git commit -sam"Bump in ${COLLECTION}: ${LIBRARY}${VERSION}"
  git push
fi




