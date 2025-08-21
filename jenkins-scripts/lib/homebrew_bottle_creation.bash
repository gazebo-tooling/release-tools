#!/bin/bash -x
set -e

[[ -L ${0} ]] && SCRIPT_LIBDIR=$(readlink ${0}) || SCRIPT_LIBDIR=${0}
SCRIPT_LIBDIR="${SCRIPT_LIBDIR%/*}"

. ${SCRIPT_LIBDIR}/_homebrew_path_setup.sh

PKG_DIR=${WORKSPACE}/pkgs

echo '# BEGIN SECTION: check variables'
if [ -z "${ghprbActualCommit}" ]; then
    echo ghprbActualCommit not specified
    exit -1
fi
if [ -z "${ghprbCommentBody}" ]; then
    echo ghprbCommentBody not specified
    exit -1
fi
if [ -z "${ghprbGhRepository}" ]; then
    echo ghprbGhRepository not specified
    exit -1
fi
if [ -z "${ghprbPullId}" ]; then
    echo ghprbPullId not specified
    exit -1
fi
if [ -z "${ghprbTargetBranch}" ]; then
    echo ghprbTargetBranch not specified
    exit -1
fi
if [ -z "${sha1}" ]; then
    echo sha1 not specified
    exit -1
fi
export GITHUB_BASE_REF=${ghprbTargetBranch}
export GITHUB_REPOSITORY=${ghprbGhRepository}
export GITHUB_REF=${sha1}
export GITHUB_SHA=${ghprbActualCommit}
MACOS_VERSION_TO_SYM=$(brew ruby -e 'puts "#{MacOS.version.to_sym}"')
if [[ "${ghprbCommentBody}" =~ 'brew-bot-tag:' ]]; then
  if [[ "${ghprbCommentBody}" =~ 'build-for-new-distro-' ]]; then
    echo Found a build-for-new-distro- option in the comment. Limiting to matching macOS versions.
    export KEEP_OLD=--keep-old
    if [[ "${ghprbCommentBody}" =~ build-for-new-distro-${MACOS_VERSION_TO_SYM} ]]; then
      echo Found a match for build-for-new-distro-${MACOS_VERSION_TO_SYM} in comment.
      echo Proceeding with bottle build.
    else
      echo Did not find --only-${MACOS_VERSION_TO_SYM} in comment string \"${ghprbCommentBody}\"
      exit 0
    fi
  fi
fi
echo '# END SECTION'

echo '# BEGIN SECTION: clean up environment'
export HOMEBREW_NO_INSTALL_FROM_API=1
rm -fr ${PKG_DIR} && mkdir -p ${PKG_DIR}
. ${SCRIPT_LIBDIR}/_homebrew_cleanup.bash
# don't use HOMEBREW_UPDATE_TO_TAG for bottle builds
unset HOMEBREW_UPDATE_TO_TAG
brew up
# manually exclude a ruby warning that jenkins thinks is from clang
# https://github.com/osrf/homebrew-simulation/issues/1343
brew install hub \
   2>&1 | grep -v 'warning: conflicting chdir during another chdir block'
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
# git to keep the slave working
export HOMEBREW_DEVELOPER=1
brew tap homebrew/test-bot
brew tap osrf/simulation
# replace with 'hub -C $(brew --repo osrf/simulation) pr checkout ${ghprbPullId}'
# after the following hub issue is resolved:
# https://github.com/github/hub/issues/2612
# hub requires authentication to operate. GITHUB_TOKEN enviroment variable for example
pushd $(brew --repo osrf/simulation) && \
  hub pr checkout ${ghprbPullId} && \
  popd

# skip tap syntax for now until we replace :x11
# do this by invoking with --only-setup and --only-formulae
brew test-bot --tap=osrf/simulation \
              --only-setup

brew test-bot --tap=osrf/simulation \
              --fail-fast \
              ${KEEP_OLD} \
              --only-formulae \
              --root-url=https://osrf-distributions.s3.amazonaws.com/bottles-simulation
echo '# END SECTION'

echo '# BEGIN SECTION: export bottle'
if [[ $(find . -name '*.bottle.*' | wc -l | sed 's/^ *//') -lt 2 ]]; then
  echo "Can not find at least two bottle files."
  # sometimes particular disabled distributions won't generate bottles,
  # --fail-fast should cover errors in bot. Do not fail
  exit 0
fi

# local bottle names don't match the uploaded names anymore
# https://github.com/Homebrew/brew/pull/4612
for j in $(ls *.bottle.json); do
  SRC_BOTTLE=$(brew ruby -e \
    "puts JSON.load(IO.read(\"${j}\")).values[0]['bottle']['tags'].values[0]['local_filename']")
  DEST_BOTTLE=$(brew ruby -e \
    "puts URI::DEFAULT_PARSER.unescape(JSON.load(IO.read(\"${j}\")).values[0]['bottle']['tags'].values[0]['filename'])")
  mv ${SRC_BOTTLE} ${DEST_BOTTLE}
done
mv *.bottle*.tar.gz ${PKG_DIR}
mv *.bottle.json ${PKG_DIR}

echo '# END SECTION'
