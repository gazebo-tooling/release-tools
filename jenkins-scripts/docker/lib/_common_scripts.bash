export APT_INSTALL="sudo DEBIAN_FRONTEND=noninteractive apt-get install -y"

_install_gzdev_repos()
{
  GZDEV_DIR="${WORKSPACE}/gzdev"
  GZDEV_CMD="${GZDEV_DIR}/gzdev.py"
  GZDEV_BRANCH=${GZDEV_BRANCH:-master}
  if python3 ${SCRIPT_DIR}/../tools/detect_ci_matching_branch.py "${ghprbSourceBranch}"; then
    GZDEV_TRY_BRANCH="${ghprbSourceBranch}"
  fi

  rm -fr "${GZDEV_DIR}" && git clone https://github.com/gazebo-tooling/gzdev -b "${GZDEV_BRANCH}" "${GZDEV_DIR}"
  if [ -n "${GZDEV_TRY_BRANCH}" ]; then
    git -C "${GZDEV_DIR}" fetch origin "$GZDEV_TRY_BRANCH" || true
    git -C "${GZDEV_DIR}" checkout "$GZDEV_TRY_BRANCH" || true
  fi || true

  git -C "${GZDEV_DIR}" branch  # print branch for informational purposes

  if [[ -n ${GZDEV_PROJECT_NAME} ]]; then
    # debian sid docker images does not return correct name so we need to use force-linux-distro
   ${GZDEV_CMD} repository enable --project="${GZDEV_PROJECT_NAME}" --force-linux-distro="${DISTRO}"
  fi

  # This could duplicate repositories enabled in the project step above. gzdev should warn about it without failing.
  for repo in ${OSRF_REPOS_TO_USE}; do
    ${GZDEV_CMD} repository enable osrf "${repo}" --force-linux-distro="${DISTRO} "
  done
}

_install_gz_source_defined_packages()
{
  # Install debian dependencies defined on the source code using packages*.apt files
  DEPENDENCIES_PATH_TO_SEARCH=${SOFTWARE_DIR:=.}
  SOURCE_DEFINED_DEPS="$(sort -u $(find ${DEPENDENCIES_PATH_TO_SEARCH} -iname 'packages-'$DISTRO'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | tr '\n' ' ')"
  ${APT_INSTALL} ${SOURCE_DEFINED_DEPS}
}
