# Parameters:
# - PULL_REQUEST_BRANCH [optional] branch to use in existing pull request 
# - PULL_REQUEST_HEAD_REPO [optional] repository with head of pull request
# Return:
# -> TAP_PREFIX

echo '# BEGIN SECTION: check variables'
if [ -z "${PULL_REQUEST_HEAD_REPO}" ]; then
  echo PULL_REQUEST_HEAD_REPO not specified, setting to osrfbuild
  echo
  PULL_REQUEST_HEAD_REPO=git@github.com:osrfbuild/homebrew-simulation.git
fi
echo '# END SECTION'

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
sh -c "$(curl -fsSL https://raw.githubusercontent.com/Linuxbrew/install/master/install.sh)"
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
${GIT} fetch --unshallow
${GIT} fetch pr_head
# change to pull request branch in case new formula is being added
if [ -n "${PULL_REQUEST_BRANCH}" ]; then
  ${GIT} checkout ${PULL_REQUEST_BRANCH}
fi
