#!/bin/bash -x

if [ -z "${PACKAGE_ALIAS}" ]; then
  echo PACKAGE_ALIAS not specified
  exit -1
fi
if [ -z "${SOURCE_TARBALL_URI}" ]; then
  echo SOURCE_TARBALL_URI not specified
  exit -1
fi
if [ -z "${VERSION}" ]; then
  echo VERSION not specified
  exit -1
fi
if [ -z "${SOURCE_TARBALL_SHA}" ]; then
  echo SOURCE_TARBALL_SHA not specified, computing now
  echo
  SOURCE_TARBALL_SHA=`curl -L ${SOURCE_TARBALL_URI} \
    | shasum --algorithm 256 \
    | awk '{print $1}'`
fi

# comment out the following two lines for faster debugging if it has already been cloned
BREW_PREFIX="${PWD}/linuxbrew"
GIT="git -C ${BREW_PREFIX}"
if ${GIT} remote -v | grep linuxbrew.git ; then
  # copying cleanup_before git commands from test-bot.rb
  echo "Cleaning up existing linuxbrew repository"
  ${GIT} gc --auto
  ${GIT} stash
  ${GIT} am --abort
  ${GIT} rebase --abort
  ${GIT} reset --hard
  ${GIT} checkout -f master
  ${GIT} clean -ffdx
  ${GIT} pull
else
  echo "Cloning new copy of linuxbrew repository"
  rm -rf linuxbrew
  git clone https://github.com/Homebrew/linuxbrew.git
fi

BREW=${PWD}/linuxbrew/bin/brew

# tap dev-tools to get brew ruby command
${BREW} tap homebrew/dev-tools
${BREW} tap osrf/simulation
TAP_PREFIX=${PWD}/linuxbrew/Library/Taps/osrf/homebrew-simulation

echo
if [ -s ${TAP_PREFIX}/${PACKAGE_ALIAS}.rb ]; then
  FORMULA=${TAP_PREFIX}/${PACKAGE_ALIAS}.rb
elif [ -s ${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS} ]; then
  FORMULA=${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS}
else
  echo Formula for ${PACKAGE_ALIAS} not found
  ls homebrew-simulation/*
  exit -1
fi

FORMULA_PATH=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.path"`
echo Modifying ${FORMULA_PATH}

echo
# get stable uri and its line number
URI=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.url"`
echo Changing url from
echo ${URI} to
echo ${SOURCE_TARBALL_URI}
URI_LINE=`grep -n ${URI} ${FORMULA_PATH} \
  | head -1 \
  | sed -e 's@:.*@@'`
echo on line number ${URI_LINE}
sed -i -e "${URI_LINE}c\  url \"${SOURCE_TARBALL_URI}\"" ${FORMULA_PATH}

echo
# get stable sha256 and its line number
SHA=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.checksum"`
echo Changing sha256 from
echo ${SHA} to
echo ${SOURCE_TARBALL_SHA}
SHA_LINE=`awk "/${SHA}/ {print FNR}" ${FORMULA_PATH} | head -1`
echo on line number ${SHA_LINE}
sed -i -e "${SHA_LINE}c\  sha256 \"${SOURCE_TARBALL_SHA}\"" ${FORMULA_PATH}

echo
GIT="git -C ${TAP_PREFIX}"
DIFF_LENGTH=`${GIT} diff | wc -l`
if [ ${DIFF_LENGTH} -eq 0 ]; then
  echo No formula modifications found, aborting
  exit -1
fi
echo ==========================================================
${GIT} diff
echo ==========================================================

echo
${GIT} remote add fork git@github.com:osrfbuild/homebrew-simulation.git
${GIT} config user.name "OSRF Build Bot"
${GIT} config user.email "osrfbuild@osrfoundation.org"
${GIT} remote -v
BRANCH="${PACKAGE_ALIAS}_${VERSION}"
${GIT} checkout -b ${BRANCH}
${GIT} commit ${FORMULA_PATH} -m "${PACKAGE_ALIAS} ${VERSION}"
echo
${GIT} status
echo
${GIT} show HEAD
echo
${GIT} push -u fork ${BRANCH}

# Check for hub command
HUB=hub
if ! which ${HUB} ; then
  if [ ! -s hub-linux-amd64-2.2.2.tgz ]; then
    echo
    echo Downloading hub...
    wget -q https://github.com/github/hub/releases/download/v2.2.2/hub-linux-amd64-2.2.2.tgz
    echo Downloaded
  fi
  HUB=`tar tf hub-linux-amd64-2.2.2.tgz | grep /hub$`
  tar xf hub-linux-amd64-2.2.2.tgz ${HUB}
  HUB=${PWD}/${HUB}
fi

# This cd needed because -C doesn't seem to work for pull-request
cd ${TAP_PREFIX}
${HUB} -C ${TAP_PREFIX} pull-request \
  -b osrf:master \
  -h osrfbuild:${BRANCH} \
  -m "${PACKAGE_ALIAS} ${VERSION}"
