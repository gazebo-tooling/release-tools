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
# tests (gzdev filter-tests). Prints nothing unless ENABLE_FILTER_TESTS=true.
#   $1: repository checkout inside the container
#   $2: job kind passed to filter_tests.bash (ci or abi)
# The image clones gzdev into /root/gzdev as root; build.sh runs as the jenkins
# user, who cannot read /root, so the checkout is copied first.
generate_buildsh_filter_tests()
{
  [[ ${ENABLE_FILTER_TESTS:-false} == true ]] || return 0
  if ! [[ ${FILTER_TESTS_NONE_RC:-} =~ ^[0-9]{1,3}$ ]] ||
     (( 10#${FILTER_TESTS_NONE_RC} < 3 || 10#${FILTER_TESTS_NONE_RC} > 125 )); then
    echo "filter-tests disabled: FILTER_TESTS_NONE_RC must be a number between 3 and 125" >&2
    return 0
  fi
  # Base 10 from here on: 099 must not be read as an invalid octal number
  local FILTER_TESTS_NONE_RC=$(( 10#${FILTER_TESTS_NONE_RC} ))
cat << DELIM_FILTER_TESTS
echo '# BEGIN SECTION: filter-tests'
sudo cp -a /root/gzdev /tmp/gzdev-filter-tests && sudo chown -R "\$(id -u):\$(id -g)" /tmp/gzdev-filter-tests || true
filter_tests_rc=0
GZDEV_DIR=/tmp/gzdev-filter-tests \\
FILTER_TESTS_NONE_RC=${FILTER_TESTS_NONE_RC} \\
ghprbTargetBranch=$(printf '%q' "${ghprbTargetBranch:-}") \\
ghprbActualCommit=$(printf '%q' "${ghprbActualCommit:-}") \\
  bash $(printf '%q' "${WORKSPACE}/scripts/jenkins-scripts/lib/filter_tests.bash") $(printf '%q' "${1}") ${2} \\
  || filter_tests_rc=\$?
echo '# END SECTION'
if [ \$filter_tests_rc -eq ${FILTER_TESTS_NONE_RC} ]; then exit 0; fi
DELIM_FILTER_TESTS
}
