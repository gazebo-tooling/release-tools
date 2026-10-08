#!/bin/bash
#
# Ask `gzdev filter-tests` whether a pull request needs its tests.
#
# Usage: filter_tests.bash <repository checkout> <job kind: ci|abi|brew>
#
# Exits ${FILTER_TESTS_NONE_RC} when the change needs no tests, after writing
# the reports the job's publishers expect. Exits 0 in every other case,
# including any problem, so the job runs its normal build.
#
# Environment:
#   FILTER_TESTS_NONE_RC (required) exit code meaning "no tests needed", 3-125
#   WORKSPACE            (required) Jenkins workspace
#   GZDEV_DIR            (optional) gzdev checkout to use. Without it gzdev is
#                        cloned into ${WORKSPACE}/gzdev-filter-tests, using
#                        the ci_matching_branch/ branch when there is one
#   GZDEV_URL            (optional) repository to clone gzdev from
#   ghprbTargetBranch, ghprbActualCommit, ghprbSourceBranch: set by ghprb

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=${1:-}
JOB_KIND=${2:-}
GZDEV_URL=${GZDEV_URL:-https://github.com/gazebo-tooling/gzdev}

build_anyway()
{
  echo "filter_tests.bash: ${1}: running the full build"
  exit 0
}

# Print the gzdev checkout to use, cloning it when GZDEV_DIR is unset
gzdev_checkout()
{
  if [[ -n ${GZDEV_DIR:-} ]]; then
    echo "${GZDEV_DIR}"
    return 0
  fi
  local dir="${WORKSPACE}/gzdev-filter-tests" branch=master
  if python3 "${SCRIPT_DIR}/../tools/detect_ci_matching_branch.py" "${ghprbSourceBranch:-}" >&2 \
     && git ls-remote --exit-code --heads "${GZDEV_URL}" "${ghprbSourceBranch}" >&2; then
    branch=${ghprbSourceBranch}
  fi
  rm -fr "${dir}"
  git clone --quiet --depth 1 --branch "${branch}" "${GZDEV_URL}" "${dir}" >&2 || return 1
  echo "${dir}"
}

# Placeholder reports for publishers that fail when a report is missing
write_reports()
{
  case ${JOB_KIND} in
    ci)
      # CppcheckPublisher is configured with allowNoReport false
      mkdir -p "${WORKSPACE}/build/cppcheck_results" || return 1
      cat > "${WORKSPACE}/build/cppcheck_results/filter_tests.xml" << 'DELIM_CPPCHECK'
<?xml version="1.0" encoding="UTF-8"?>
<results version="2">
  <cppcheck version="2.0"/>
  <errors>
  </errors>
</results>
DELIM_CPPCHECK
      ;;
    abi)
      # The HTML publisher of abi jobs is configured with allowMissing false
      rm -fr "${WORKSPACE}/reports" && mkdir -p "${WORKSPACE}/reports" || return 1
      echo '<html><body><p>ABI check skipped: no tests needed for this change.</p></body></html>' \
        > "${WORKSPACE}/reports/compat_report.html"
      ;;
  esac
}

if ! [[ ${FILTER_TESTS_NONE_RC:-} =~ ^[0-9]{1,3}$ ]] ||
   (( 10#${FILTER_TESTS_NONE_RC} < 3 || 10#${FILTER_TESTS_NONE_RC} > 125 )); then
  build_anyway "FILTER_TESTS_NONE_RC must be a number between 3 and 125"
fi
# Base 10 from here on: 099 must not be read as an invalid octal number
FILTER_TESTS_NONE_RC=$(( 10#${FILTER_TESTS_NONE_RC} ))
if [[ -z ${REPO_DIR} || -z ${WORKSPACE:-} ]]; then
  build_anyway "usage: filter_tests.bash <repository checkout> <job kind> with WORKSPACE set"
fi

gzdev_dir=$(gzdev_checkout) || build_anyway "cannot get gzdev"
# Reports left by an earlier build must not be published next to the stub
rm -fr "${WORKSPACE}/build/test_results"
rc=0
python3 "${gzdev_dir}/gzdev.py" filter-tests --build-platform=jenkins \
  --repo-path="${REPO_DIR}" --return-if-none="${FILTER_TESTS_NONE_RC}" \
  --junit-output="${WORKSPACE}/build/test_results/filter_tests.xml" || rc=$?

if [[ ${rc} -eq ${FILTER_TESTS_NONE_RC} ]]; then
  write_reports || build_anyway "cannot write the placeholder reports"
  echo "filter_tests.bash: no tests needed for this change"
  exit "${FILTER_TESTS_NONE_RC}"
fi
if [[ ${rc} -ne 0 ]]; then
  build_anyway "gzdev filter-tests could not decide (rc ${rc})"
fi
exit 0
