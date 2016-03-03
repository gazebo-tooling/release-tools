#!/bin/bash -xe

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: sending bitbucket status: inprogress'
NEEDED_HOST_PACKAGES="python-yaml python-six python-uritemplate python-requests python-requests-oauthlib python-pip"
REPO_SHORT_NAME=`echo ${SRC_REPO} | sed s:.*\.org/::`

# Source bitbucket configs
. ${SCRIPT_DIR}/_bitbucket_configs.bash

if [[ ! -f ${BITBUCKET_USER_PASS_FILE} ]]; then
  echo "Bitbucket user pass not found in file \${BITBUCKET_USER_PASS_FILE}"
  exit 1
fi

# Check if they are already installed in the host. 
# dpkg-query will return an error in stderr if a package has never been in the
# system. It will return a header composed by several lines started with |, +++
# and 'Desired' the rest of lines is composed by: ^rc or ^un if the package is
# not in the system. ^in if it is installed
QUERY_RESULT=$(dpkg-query --list ${NEEDED_HOST_PACKAGES} 2>&1 | grep -v ^ii | grep -v '|' | grep -v '^\+++' | grep -v '^Desired') || true
if [[ -n ${QUERY_RESULT} ]]; then
  # Trick to not run apt-get update if there is no error in installation
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES} || { sudo apt-get update && sudo apt-get install -y ${NEEDED_HOST_PACKAGES}; }
  # python-future package is available only from Ubuntu wily on
  pip install future
fi

echo "Generating config file ..."
cat > $BITBUCKET_BUILD_STATUS_FILE << DELIM_CONFIG
bitbucket_origin:
  repository_name: ${REPO_SHORT_NAME}
  sha: ${MERCURIAL_REVISION_SHORT}
jenkins_job:
  name: ${JOB_NAME}
  url: ${BUILD_URL}
DELIM_CONFIG
cat $BITBUCKET_BUILD_STATUS_FILE
