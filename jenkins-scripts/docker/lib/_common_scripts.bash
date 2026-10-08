export APT_INSTALL="sudo DEBIAN_FRONTEND=noninteractive apt-get install -y"

generate_buildsh_header()
{
  SHELL_ON_ERRORS=${SHELL_ON_ERRORS:-false}
  echo "#!/bin/bash"
  echo "set -ex"
  if ${SHELL_ON_ERRORS}; then
    echo 'trap "/bin/bash" 0 INT QUIT ABRT PIPE TERM'
  fi
  if $GENERIC_ENABLE_TIMING; then
    echo "source ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}"
  fi
}

# Print build.sh lines that end the build early when the pull request needs no
# build or tests (gzdev filter-build-and-test). Prints nothing unless
# ENABLE_FILTER_BUILD_AND_TEST=true.
#   $1: repository checkout inside the container
#   $2: job kind passed to filter_build_and_test.bash (ci or abi)
# The image clones gzdev into /root/gzdev as root; build.sh runs as the jenkins
# user, who cannot read /root, so the checkout is copied first.
generate_buildsh_filter_build_and_test()
{
  [[ ${ENABLE_FILTER_BUILD_AND_TEST:-false} == true ]] || return 0
  if ! [[ ${FILTER_BUILD_AND_TEST_NONE_RC:-} =~ ^[0-9]+$ ]]; then
    echo "filter-build-and-test disabled: FILTER_BUILD_AND_TEST_NONE_RC is not a number" >&2
    return 0
  fi
cat << DELIM_FILTER_BUILD_AND_TEST
echo '# BEGIN SECTION: filter-build-and-test'
sudo cp -a /root/gzdev /tmp/gzdev-filter-build-and-test && sudo chown -R "\$(id -u):\$(id -g)" /tmp/gzdev-filter-build-and-test || true
filter_build_and_test_rc=0
GZDEV_DIR=/tmp/gzdev-filter-build-and-test \\
FILTER_BUILD_AND_TEST_NONE_RC=${FILTER_BUILD_AND_TEST_NONE_RC} \\
ghprbTargetBranch=$(printf '%q' "${ghprbTargetBranch:-}") \\
ghprbActualCommit=$(printf '%q' "${ghprbActualCommit:-}") \\
  bash $(printf '%q' "${WORKSPACE}/scripts/jenkins-scripts/lib/filter_build_and_test.bash") $(printf '%q' "${1}") ${2} \\
  || filter_build_and_test_rc=\$?
echo '# END SECTION'
if [ \$filter_build_and_test_rc -eq ${FILTER_BUILD_AND_TEST_NONE_RC} ]; then exit 0; fi
DELIM_FILTER_BUILD_AND_TEST
}
