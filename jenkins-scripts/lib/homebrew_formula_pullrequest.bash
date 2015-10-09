#!/bin/bash -x

# rm -rf linuxbrew
# git clone https://github.com/Homebrew/linuxbrew.git
BREW=${PWD}/linuxbrew/bin/brew

${BREW} tap osrf/simulation
TAP_PREFIX=${PWD}/linuxbrew/Library/Taps/osrf/homebrew-simulation

if [ -z "${PACKAGE_ALIAS}" ]; then
  PACKAGE_ALIAS=sdformat2
fi
if [ -z "${SOURCE_TARBALL_URI}" ]; then
  SOURCE_TARBALL_URI=SOURCE_TARBALL_URI
fi
if [ -z "${SOURCE_TARBALL_SHA}" ]; then
  SOURCE_TARBALL_SHA=SOURCE_TARBALL_SHA
fi

if [ -s ${TAP_PREFIX}/${PACKAGE_ALIAS}.rb ]; then
  FORMULA=${TAP_PREFIX}/${PACKAGE_ALIAS}.rb
elif [ -s ${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS} ]; then
  FORMULA=${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS}
else
  echo Formula for ${PACKAGE_ALIAS} not found
  ls homebrew-simulation/*
  exit -1
fi

TOKEN=__TOKEN__
FORMULA_PATH1=`echo "puts '__TOKEN__' + \"${PACKAGE_ALIAS}\".f.path" \
  | ${BREW} irb \
  | grep ${TOKEN}`
FORMULA_PATH=`echo ${FORMULA_PATH1} | sed -e "s@.*${TOKEN}@@g"`
echo Modifying ${FORMULA_PATH}

# change stable uri
OLD_URI1=`echo "puts '__TOKEN__' + \"${PACKAGE_ALIAS}\".f.stable.url" \
  | ${BREW} irb \
  | grep ${TOKEN}`
OLD_URI=`echo ${OLD_URI1} | sed -e "s@.*${TOKEN}@@g"`
OLD_URI_LINE=`${BREW} cat ${PACKAGE_ALIAS} \
  | grep -n ${OLD_URI} \
  | head -1 \
  | sed -e 's@:.*@@'`
echo
echo Changing url from
echo ${OLD_URI} to
echo ${SOURCE_TARBALL_URI}
sed -i -e "${OLD_URI_LINE}c\  url \"${SOURCE_TARBALL_URI}\"" \
  ${FORMULA_PATH}

# change stable uri
OLD_SHA1=`echo "puts '__TOKEN__' + \"${PACKAGE_ALIAS}\".f.stable.checksum.to_s" \
  | ${BREW} irb \
  | grep ${TOKEN}`
OLD_SHA=`echo ${OLD_SHA1} | sed -e "s@.*${TOKEN}@@g"`
OLD_SHA_LINE=`${BREW} cat ${PACKAGE_ALIAS} \
  | grep -n ${OLD_SHA} \
  | head -1 \
  | sed -e 's@:.*@@'`
echo
echo Changing sha256 from
echo ${OLD_SHA} to
echo ${SOURCE_TARBALL_SHA}
sed -i -e "${OLD_SHA_LINE}c\  sha256 \"${SOURCE_TARBALL_URI}\"" \
  ${FORMULA_PATH}
