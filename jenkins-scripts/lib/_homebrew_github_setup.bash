# Parameters:
# - PACKAGE_ALIAS [mandatory] name of package including major version

echo '# BEGIN SECTION: check github perms'
# Github autentication. git access is provided by public key access
# and hub cli needs a token
if [[ -z $(ssh -T git@github.com 2>&1 | grep successfully) ]]; then
    echo "The github connection seems not to be valid:"
    ssh -T git@github.com
    echo "Please check that the ssh key authentication is working"
    exit 1
fi

GITHUB_TOKEN_FILE="/var/lib/jenkins/.github_token"
if [[ ! -f ${GITHUB_TOKEN_FILE} ]]; then
   echo "The hub cli tool needs a valid token at file ${GITHUB_TOKEN_FILE}"
   echo "The file was not found"
   exit 1
fi

set +x # keep password secret
export GITHUB_TOKEN=`cat $GITHUB_TOKEN_FILE`
set -x # back to debug
echo '# END SECTION'

echo '# BEGIN SECTION: download linuxbrew'
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
echo '# END SECTION'

BREW=${PWD}/linuxbrew/bin/brew

# tap dev-tools to get brew ruby command
${BREW} tap homebrew/dev-tools
${BREW} tap osrf/simulation
TAP_PREFIX=${PWD}/linuxbrew/Library/Taps/osrf/homebrew-simulation

echo '# BEGIN SECTION: check if the formula exists'
echo
if [ -s ${TAP_PREFIX}/${PACKAGE_ALIAS}.rb ]; then
  FORMULA=${TAP_PREFIX}/${PACKAGE_ALIAS}.rb
elif [ -s ${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS} ]; then
  FORMULA=${TAP_PREFIX}/Aliases/${PACKAGE_ALIAS}
else
  echo Formula for ${PACKAGE_ALIAS} not found
  [[ -d homebrew-simulation ]] && ls homebrew-simulation/*
  exit 0
fi
echo '# END SECTION'

echo '# BEGIN SECTION: get formula path and version'
FORMULA_PATH=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.path"`
echo Modifying ${FORMULA_PATH}

# check if formula has auto-generated version field
if ${BREW} ruby -e "exit \"${PACKAGE_ALIAS}\".f.stable.version.detected_from_url?"
then
  echo Version is autodetected from URL
else
  # get version and line number
  FORMULA_VERSION=`${BREW} ruby -e "puts \"${PACKAGE_ALIAS}\".f.stable.version"`
  echo Changing version from
  echo ${FORMULA_VERSION} to
  echo ${VERSION}
  VERSION_LINE=`awk \
    "/version .${FORMULA_VERSION}/ {print FNR}" ${FORMULA_PATH} | head -1`
  echo on line number ${VERSION_LINE}
  sed -i -e "${VERSION_LINE}c\  version \"${VERSION}\"" ${FORMULA_PATH}
fi
echo '# END SECTION'
