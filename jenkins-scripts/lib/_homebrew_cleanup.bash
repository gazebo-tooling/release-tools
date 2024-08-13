#/bin/bash +x
set -e

restore_brew()
{
    rm -fr /usr/local/Homebrew/Library/Homebrew/vendor/bundle/ruby
    ${BREW_BINARY} update-reset
    ${BREW_BINARY} vendor-install ruby
}

BREW_BINARY_DIR=/usr/local/bin
BREW_BINARY=${BREW_BINARY_DIR}/brew
# Try running `git fsck` before `brew update` in case `git gc` broke something
# but don't fail
git -C $(${BREW_BINARY} --repo) fsck || true
export HOMEBREW_UPDATE_TO_TAG=1
${BREW_BINARY} up || { restore_brew && ${BREW_BINARY} up ; }

# Clear all installed homebrew packages, links, taps, and kegs
${BREW_BINARY} list --formula > /dev/null || { restore_brew && ${BREW_BINARY} list --formula > /dev/null; }
BREW_LIST=$(${BREW_BINARY} list --formula)
if [[ -n "${BREW_LIST}" ]]; then
  ${BREW_BINARY} remove --force --ignore-dependencies ${BREW_LIST}
fi
rm -rf /usr/local/lib/python*/site-packages
hash -r
# redirect error to /dev/null to avoid temporal problems detected by
# brew tap
for t in $(HOMEBREW_NO_AUTO_UPDATE=1 \
          ${BREW_BINARY} tap 2>/dev/null \
          | grep '^[^/]\+/[^/]\+$' \
          | grep -v '^homebrew/cask$' \
          | grep -v '^homebrew/core$'); do
  ${BREW_BINARY} untap $t
done
${BREW_BINARY} cleanup --prune-prefix

pushd $(${BREW_BINARY} --prefix)/Homebrew/Library 2> /dev/null
git stash && git clean -d -f
# Need to test if brew installation is still working (use audit cmake to quick check)
${BREW_BINARY} audit cmake || restore_brew
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
