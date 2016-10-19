# Parameters:
# - PULL_REQUEST_BRANCH [optional] branch to use in existing pull request 
# Return:
# -> TAP_PREFIX

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
if ${GIT} remote -v | grep Linuxbrew/brew.git ; then
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
  rm -rf brew
  git clone https://github.com/Linuxbrew/brew.git
fi
echo '# END SECTION'

BREW=${PWD}/brew/bin/brew
${BREW} up

# tap dev-tools to get brew ruby command
${BREW} tap homebrew/dev-tools
${BREW} ruby -e "puts 'brew ruby success'"

# tap osrf/simulation
${BREW} tap osrf/simulation
TAP_PREFIX=${PWD}/brew/Library/Taps/osrf/homebrew-simulation
GIT="git -C ${TAP_PREFIX}"
${GIT} remote add fork git@github.com:osrfbuild/homebrew-simulation.git
# unshallow to get a full clone able to push
${GIT} fetch --unshallow
${GIT} fetch fork
# change to pull request branch in case new formula is being added
if [ -n "${PULL_REQUEST_BRANCH}" ]; then
  ${GIT} checkout ${PULL_REQUEST_BRANCH}
fi
