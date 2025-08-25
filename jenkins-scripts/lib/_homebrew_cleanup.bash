#/bin/bash +x
set -e

restore_brew()
{
    rm -fr ${HOMEBREW_REPOSITORY}/Library/Homebrew/vendor/bundle/ruby
    brew update-reset
    brew vendor-install ruby
}

# Try running `git fsck` before `brew update` in case `git gc` broke something
# but don't fail
git -C $(brew --repo) fsck || true
export HOMEBREW_UPDATE_TO_TAG=1
# Assume that brew is already in the PATH
brew up || { restore_brew && brew up ; }
if ${CLEAR_BREW_CACHE}; then
  restore_brew && brew up
fi

# Clear all installed homebrew packages, links, taps, and kegs
brew list --formula > /dev/null || { restore_brew && brew list --formula > /dev/null; }
BREW_LIST=$(brew list --formula)
if [[ -n "${BREW_LIST}" ]]; then
  brew remove --force --ignore-dependencies ${BREW_LIST}
fi
rm -rf ${HOMEBREW_PREFIX}/lib/python*/site-packages
hash -r
# redirect error to /dev/null to avoid temporal problems detected by
# brew tap
for t in $(HOMEBREW_NO_AUTO_UPDATE=1 \
          brew tap 2>/dev/null \
          | grep '^[^/]\+/[^/]\+$' \
          | grep -v '^homebrew/cask$' \
          | grep -v '^homebrew/core$'); do
  brew untap $t
done
brew cleanup --prune-prefix

BREW_CACHE=$(brew --cache)
echo BREW_CACHE=${BREW_CACHE}
if ${CLEAR_BREW_CACHE}; then
  rm -rf ${BREW_CACHE}
fi

pushd ${HOMEBREW_REPOSITORY}/Library 2> /dev/null
git stash && git clean -d -f
# Need to test if brew installation is still working (use audit cmake to quick check)
brew audit cmake || restore_brew
popd 2> /dev/null

# test-bot needs variables and does not work just with config not sure why
export GIT_AUTHOR_NAME="OSRF Build Bot"
export GIT_COMMITTER_NAME=${GIT_AUTHOR_NAME}
export HOMEBREW_GIT_NAME=${GIT_AUTHOR_NAME}
export GIT_AUTHOR_EMAIL="osrfbuild@osrfoundation.org"
export GIT_COMMITTER_EMAIL=${GIT_AUTHOR_EMAIL}
export HOMEBREW_GIT_EMAIL=${GIT_AUTHOR_EMAIL}
# Cleanup any existing lock left behind from aborted builds
# see https://github.com/osrf/buildfarmer/issues/257
rm -f ${HOME}/.gitconfig.lock
git config --global user.name "${GIT_AUTHOR_NAME}"
git config --global user.email "${GIT_AUTHOR_EMAIL}"
