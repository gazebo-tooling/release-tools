#!/bin/bash -x
set -e

[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

export PATH="/usr/local/bin:$PATH"

PKG_DIR=${WORKSPACE}/pkgs

echo '# BEGIN SECTION: check variables'
if [ -z "${PULL_REQUEST_URL}" ]; then
    echo PULL_REQUEST_URL not specified
    exit -1
fi
echo '# END SECTION'

echo '# BEGIN SECTION: clean up environment'
rm -fr ${PKG_DIR} && mkdir -p ${PKG_DIR}
. ${SCRIPT_LIBDIR}/_homebrew_cleanup.bash
echo '# END SECTION'

# set display before building bottle
# search for Xquartz instance owned by current user
export DISPLAY=$(ps ax \
  | grep '[[:digit:]]*:[[:digit:]][[:digit:]].[[:digit:]][[:digit:]] /opt/X11/bin/Xquartz' \
  | grep "auth /Users/$(whoami)/" \
  | sed -e 's@.*Xquartz @@' -e 's@ .*@@'
)

echo '# BEGIN SECTION: run test-bot'
# The test-bot makes a full cleanup of all installed pkgs. Be sure of install back
# mercurial to keep the slave working
export HOMEBREW_DEVELOPER=1
brew tap homebrew/test-bot
git -C $(brew --repo)/Library/Taps/homebrew/homebrew-test-bot \
    fetch ${TEST_BOT_REPO} ${TEST_BOT_BRANCH}
git -C $(brew --repo)/Library/Taps/homebrew/homebrew-test-bot \
    checkout FETCH_HEAD

git -C $(brew --repo) fetch ${BREW_REPO} ${BREW_BRANCH}
git -C $(brew --repo) checkout FETCH_HEAD

brew test-bot --tap=osrf/simulation \
              --root-url=https://osrf-distributions.s3.amazonaws.com/bottles-simulation \
              --ci-pr ${PULL_REQUEST_URL} \
            || { brew install hg; exit -1; }
brew install hg
echo '# END SECTION'

echo '# BEGIN SECTION: export bottle'
if [[ $(find . -name '*.bottle.*' | wc -l | sed 's/^ *//') -lt 2 ]]; then
  echo "Can not find at least two bottle files."
  exit 0
fi

# local bottle names don't match the uploaded names anymore
# https://github.com/Homebrew/brew/pull/4612
for j in $(ls *.bottle.json); do
  SRC_BOTTLE=$(brew ruby -e \
    "puts JSON.load(IO.read(\"${j}\")).values[0]['bottle']['tags'].values[0]['local_filename']")
  DEST_BOTTLE=$(brew ruby -e \
    "puts URI.decode(JSON.load(IO.read(\"${j}\")).values[0]['bottle']['tags'].values[0]['filename'])")
  mv ${SRC_BOTTLE} ${DEST_BOTTLE}
done
mv *.bottle*.tar.gz ${PKG_DIR}
mv *.bottle.json ${PKG_DIR}

echo '# END SECTION'
