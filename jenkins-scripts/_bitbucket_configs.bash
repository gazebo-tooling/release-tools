# Variables exported by the system
# BUILD_STATUS_FILE need to be defined here if scripts are used from our current scripts and not as a job
if [[ -z ${BITBUCKET_BUILD_STATUS_FILE} ]]; then
  export BITBUCKET_BUILD_STATUS_FILE="$WORKSPACE/config_pybitbucket.yml"
fi
export BITBUCKET_USER_PASS_FILE="/var/lib/jenkins/osrf_jenkins_bitbucket_user_pass"
export BITBUCKET_LOG_FILE="${WORKSPACE}/pybitbucket.log"
