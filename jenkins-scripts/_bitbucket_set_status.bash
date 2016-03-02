#!/bin/bash -xe

echo '# BEGIN SECTION: sending bitbucket status: inprogress'
export NEEDED_HOST_PACKAGES="python-yaml python-six python-uritemplate python-requests python-requests-oauthlib python-pip"
export CONFIG_FILE_PATH="$WORKSPACE/config_pybitbucket.yml"
export BITBUCKET_USER_PASS_FILE="/var/lib/jenkins/osrf_jenkins_bitbucket_user_pass"
export REPO_SHORT_NAME=`echo ${SRC_REPO} | sed s:.*\.org/::`

if [[ ! -f ${BITBUCKET_USER_PASS_FILE} ]]; then
  echo "Bitbucket user pass not found in file \${BITBUCKET_USER_PASS_FILE}"
  exit 0
fi

# Check if they are already installed in the host. 
# dpkg-query will return an error in stderr if a package has never been in the
# system. It will return a header composed by several lines started with |, +++
# and 'Desired' the rest of lines is composed by: ^rc or ^un if the package is
# not in the system. ^in if it is installed
QUERY_RESULT=$(dpkg-query --list g++ 2>&1 | grep -v ^ii | grep -v '|' | grep -v '^\+++' | grep -v '^Desired')
if [[ -z ${QUERY_RESULT} ]]; then
  # Trick to not run apt-get update if there is no error in installation
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES} || { sudo apt-get update && sudo apt-get install -y ${NEEDED_HOST_PACKAGES}; }
  # python-future package is available only from Ubuntu wily on
  pip install future
fi

echo "Generating config file ..."
cat > $CONFIG_FILE_PATH << DELIM_CONFIG
bitbucket_origin:
  repository_name: ${REPO_SHORT_NAME}
  sha: ${MERCURIAL_REVISION_SHORT}
jenkins_job:
  name: ${JOB_NAME}
  url: ${BUILD_URL}
DELIM_CONFIG
cat $CONFIG_FILE_PATH

echo "Running the set_status_from_file in hidden mode"
set +x # keep password secret
BITBUCKET_USER_PASS=`cat $BITBUCKET_USER_PASS_FILE`
${WORKSPACE}/scripts/jenkins-scripts/python-bitbucket/set_status_from_file.py \
    --user osrf_jenkins  \
    --pass ${BITBUCKET_USER_PASS} \
    --status inprogress \
    --load_from_file ${CONFIG_FILE_PATH} >& pybitbucket.log
set -x # back to debug
echo '# END SECTION'
