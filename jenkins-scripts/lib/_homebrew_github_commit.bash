# parameters:
# - TAP_PREFIX
# - PULL_REQUEST_BRANCH
# - PACKAGE_ALIAS
# - VERSION

# Can be defined outside the script. if not, default value is set
PR_URL_export_file=${PR_URL_export_file:-${WORKSPACE}/pull_request_created.properties}

echo '# BEGIN SECTION: check variables'
if [ -z "${COMMIT_MESSAGE}" ]; then
  echo COMMIT_MESSAGE not specified
  exit -1
fi
if [ -z "${PULL_REQUEST_BRANCH}" ]; then
  echo PULL_REQUEST_BRANCH not specified
  exit -1
fi
if [ -z "${PULL_REQUEST_TITLE}" ]; then
  if [ -z "${PULL_REQUEST_URL}" ]; then
    echo One of PULL_REQUEST_TITLE or PULL_REQUEST_URL must be specified
    exit -1
  fi
fi
if [ -z "${TAP_PREFIX}" ]; then
  echo TAP_PREFIX not specified
  exit -1
fi
echo '# END SECTION'

GIT="git -C ${TAP_PREFIX}"

echo '# BEGIN SECTION: commit and pull request creation'
echo
${GIT} remote -v
# check if branch already exists
if ${GIT} rev-parse --verify ${PULL_REQUEST_BRANCH} ; then
  ${GIT} checkout ${PULL_REQUEST_BRANCH}
else
  ${GIT} checkout -b ${PULL_REQUEST_BRANCH}
fi
if [ -z "${SKIP_COMMIT}" ]; then
  DIFF_LENGTH=`${GIT} diff | wc -l`
  if [ ${DIFF_LENGTH} -eq 0 ]; then
    echo No formula modifications found, aborting
    exit -1
  fi
  echo ==========================================================
  ${GIT} diff
  echo ==========================================================

  ${GIT} commit ${FORMULA_PATH} -m "${COMMIT_MESSAGE}"
fi
echo
${GIT} status
echo
${GIT} show HEAD
echo
${GIT} push -u pr_head ${PULL_REQUEST_BRANCH}


# Create a pull request if one doesn't yet exist
if [ -z "${PULL_REQUEST_URL}" ]; then
  # Check for hub command. It requires a GITHUB_TOKEN to work
  HUB=hub
  if ! which ${HUB} ; then
    if [ ! -s hub-linux-amd64-2.2.3.tgz ]; then
      echo
      echo Downloading hub...
      wget -q https://github.com/github/hub/releases/download/v2.2.3/hub-linux-amd64-2.2.3.tgz
      echo Downloaded
    fi
    HUB=`tar tf hub-linux-amd64-2.2.3.tgz | grep /hub$`
    tar xf hub-linux-amd64-2.2.3.tgz ${HUB}
    HUB=${PWD}/${HUB}
  fi

  PR_URL=$(${HUB} -C ${TAP_PREFIX} pull-request \
    -b osrf:master \
    -h osrfbuild:${PULL_REQUEST_BRANCH} \
    -m "${PULL_REQUEST_TITLE}")

  echo "Pull request created: ${PR_URL}"

  # Exporting URL as an artifact (it will be used in other jobs)
  echo "PULL_REQUEST_URL=${PR_URL}" > ${PR_URL_export_file}
  echo "PULL_REQUEST_BRANCH=${PULL_REQUEST_BRANCH}" >> ${PR_URL_export_file}
else
  echo "Pull request updated: ${PULL_REQUEST_URL}"
fi
echo '# END SECTION'
