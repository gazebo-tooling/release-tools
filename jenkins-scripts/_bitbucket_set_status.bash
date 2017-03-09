#!/bin/bash -xe

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# STATUS can be taken an env variable or via the first argument
# when calling the script
BITBUCKET_STATUS=${BITBUCKET_STATUS:-$1}

# Source bitbucket configs
. ${SCRIPT_DIR}/_bitbucket_configs.bash

echo '# BEGIN SECTION: check that all variables are set'
if [[ -z ${JENKINS_BUILD_HG_HASH} ]]; then
  echo "JENKINS_BUILD_HG_HASH variable missing"
  exit 1
fi

if [[ -z ${JENKINS_BUILD_REPO} ]]; then
  echo "JENKINS_BUILD_REPO variable missing"
  exit 1
fi

if [[ ! -f ${BITBUCKET_USER_PASS_FILE} ]]; then
  echo "Bitbucket user pass not found in file \${BITBUCKET_USER_PASS_FILE}"
  exit 1
fi

if [[ -z $BITBUCKET_STATUS ]]; then
    echo "BITBUCKET_STATUS variable is not set. "
    echo "Can be called as a first argument of the script or via env variable"
    exit 1
fi

if [[ ! -f ${BITBUCKET_USER_PASS_FILE} ]]; then
  echo "Bitbucket user pass not found in file: ${BITBUCKET_USER_PASS_FILE}"
  exit 1
fi
echo '# END SECTION'

echo '# BEGIN SECTION: install needed software'
NEEDED_HOST_PACKAGES="python-pip"

REPO_SHORT_NAME=`echo ${JENKINS_BUILD_REPO} | sed s:.*\.org/::`

echo "Generating requirements.txt for pip"
cat > ${WORKSPACE}/requirements.txt <<- REQ
pyyaml
six
uritemplate
requests>=2.4.2
requests-oauthlib
future
REQ

# Check if they are already installed in the host.
# dpkg-query will return an error in stderr if a package has never been in the
# system. It will return a header composed by several lines started with |, +++
# and 'Desired' the rest of lines is composed by: ^rc or ^un if the package is
# not in the system. ^in if it is installed
QUERY_RESULT=$(dpkg-query --list ${NEEDED_HOST_PACKAGES} 2>&1 | grep -v ^ii | grep -v '|' | grep -v '^\+++' | grep -v '^Desired') || true
if [[ -n ${QUERY_RESULT} ]]; then
  # Trick to not run apt-get update if there is no error in installation
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES} || { sudo apt-get update && sudo apt-get install -y ${NEEDED_HOST_PACKAGES}; }
fi

sudo pip install --quiet -r ${WORKSPACE}/requirements.txt
echo '# END SECTION'

echo '# BEGIN SECTION: create bitbucket build status file'
cat > $BITBUCKET_BUILD_STATUS_FILE << DELIM_CONFIG
bitbucket_origin:
  repository_name: ${REPO_SHORT_NAME}
  sha: ${JENKINS_BUILD_HG_HASH}
jenkins_job:
  name: ${JENKINS_BUILD_JOB_NAME}
  url: ${JENKINS_BUILD_URL}
DELIM_CONFIG
cat $BITBUCKET_BUILD_STATUS_FILE
echo '# END SECTION'

echo "BEGIN SECTION: build status in bitbucket: ${BITBUCKET_STATUS} (hidden)"
set +x # keep password secret
BITBUCKET_USER_PASS=$(cat ${BITBUCKET_USER_PASS_FILE})
BITBUCKET_API_RESULT=true
${WORKSPACE}/scripts/jenkins-scripts/python-bitbucket/set_status_from_file.py \
    --user "osrf_jenkins"  \
    --pass "${BITBUCKET_USER_PASS}" \
    --desc "${JENKINS_BUILD_DESC}" \
    --status "${BITBUCKET_STATUS}" \
    --load_from_file "${BITBUCKET_BUILD_STATUS_FILE}" >& ${BITBUCKET_LOG_FILE} || BITBUCKET_API_RESULT=false
set -x # back to debug
echo '# END SECTION'

REPO_ORG=${REPO_SHORT_NAME%/*}

if ! $BITBUCKET_API_RESULT; then
  # Check if we expect the failure due to lack of permissions
  case ${REPO_ORG} in
   'osrf' | 'ignitionrobotics')
       # let the job to fail. It should not be problem with osrf repositories
       ;;
    *)
      if [[ -n $(grep '403 Client Error: Forbidden' ${BITBUCKET_LOG_FILE}) ]]; then
        echo "MARK_AS_UNSTABLE"
        exit 0
      fi
  esac

  echo 'BEGIN SECTION: build status FAILED'
  echo 'The call from the python client to bitbucket to set the build status failed'
  echo "Please check out the workspace for the file: ${BITBUCKET_LOG_FILE}"
  echo '# END SECTION'
  exit 1
fi
