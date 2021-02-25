# Parameters:
# - PULL_REQUEST_BRANCH [optional] branch to use in existing pull request 
# - PULL_REQUEST_HEAD_REPO [optional] repository with head of pull request
# Return:
# -> TAP_PREFIX

echo '# BEGIN SECTION: check variables'
if [ -z "${PULL_REQUEST_HEAD_REPO}" ]; then
  echo PULL_REQUEST_HEAD_REPO not specified, setting to osrfbuild
  echo
  # personal tokens only support https
  PULL_REQUEST_HEAD_REPO=https://github.com/osrfbuild/homebrew-simulation.git
fi
echo '# END SECTION'

echo '# BEGIN SECTION: download linuxbrew'
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
echo '# END SECTION'

BREW_PREFIX="/home/linuxbrew/.linuxbrew"
BREW=${BREW_PREFIX}/bin/brew
${BREW} up

${BREW} ruby -e "puts 'brew ruby success'"

# tap osrf/simulation
${BREW} untap osrf/simulation || true
${BREW} tap osrf/simulation
TAP_PREFIX=$(${BREW} --repo osrf/simulation)
GIT="git -C ${TAP_PREFIX}"
${GIT} remote add pr_head ${PULL_REQUEST_HEAD_REPO}
# unshallow to get a full clone able to push
${GIT} fetch --unshallow || true
${GIT} fetch pr_head
# change to pull request branch in case new formula is being added
if [ -n "${PULL_REQUEST_BRANCH}" ]; then
  ${GIT} checkout --track pr_head/${PULL_REQUEST_BRANCH}
fi
