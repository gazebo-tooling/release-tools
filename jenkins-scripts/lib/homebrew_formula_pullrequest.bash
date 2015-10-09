#!/bin/bash -x

# comment out the following two lines for faster debugging if it has already been cloned
rm -rf linuxbrew
git clone https://github.com/Homebrew/linuxbrew.git
BREW=${PWD}/linuxbrew/bin/brew

# tap dev-tools to get brew ruby command
${BREW} tap homebrew/dev-tools
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

FORMULA_PATH=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.path"`
echo Modifying ${FORMULA_PATH}

# change stable uri
OLD_URI=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.url"`
OLD_URI_LINE=`awk "index(\$0, \"${OLD_URI}\") != 0 { print FNR }" ${FORMULA_PATH} | head -1`
echo
echo Changing url from
echo ${OLD_URI} to
echo ${SOURCE_TARBALL_URI}
sed -i -e "${OLD_URI_LINE}c\  url \"${SOURCE_TARBALL_URI}\"" ${FORMULA_PATH}

# change stable sha256
OLD_SHA=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.checksum"`
OLD_SHA_LINE=`awk "/${OLD_SHA}/ {print FNR}" ${FORMULA_PATH} | head -1`
echo
echo Changing sha256 from
echo ${OLD_SHA} to
echo ${SOURCE_TARBALL_SHA}
sed -i -e "${OLD_SHA_LINE}c\  sha256 \"${SOURCE_TARBALL_SHA}\"" \
  ${FORMULA_PATH}
