#!/bin/sh
# Usage: check_pr_skip.sh <lib_name> [pattern_file ...]
# Checks whether every file changed in the PR matches the ignore patterns.
# Skipping is opt-in: the PR description must contain SKIP_CI_FOR_IGNORED_FILES.
# Exit 0: all changed files ignored; dummy test/cppcheck/ABI reports generated (skip build).
# Exit 1: build required (non-PR build, keyword absent, non-ignored files).
# Exit 2: usage error (missing lib_name) or check error. Callers should treat non-zero as "build".

set -e

LIB_NAME="${1:-}"

if [ -z "${LIB_NAME}" ]; then
  echo "Usage: $0 <lib_name> [pattern_file1 pattern_file2 ...]" >&2
  exit 2
fi

# $1 has been parsed as LIB_NAME, so it can now be dropped
shift

# Pattern files must be passed with "-i" to check_ignored_files.py
# This iterates over pattern files passed in $@ and appends to
# IGNORE_ARGS with "-i"
IGNORE_ARGS=""
if [ "$#" -gt 0 ]; then
  for pf in "$@"; do
    IGNORE_ARGS="${IGNORE_ARGS} -i ${pf}"
  done
else
  # no pattern files were passed, so use gz_ci.ignored by default
  IGNORE_ARGS="-i ./scripts/jenkins-scripts/tools/gz_ci.ignored"
fi

if [ -z "${ghprbTargetBranch:-}" ] || [ -z "${ghprbActualCommit:-}" ]; then
  echo "Not a PR build or environment variables missing; do not skip build"
  exit 1
fi

SKIP_KEYWORD="SKIP_CI_FOR_IGNORED_FILES"
case "${ghprbPullLongDescription:-}" in
  *"${SKIP_KEYWORD}"*)
    ;;
  *)
    echo "Keyword '${SKIP_KEYWORD}' not present in PR description; do not skip build"
    exit 1
    ;;
esac

REPO_DIR="${WORKSPACE}/${LIB_NAME}"

if [ ! -d "${REPO_DIR}" ]; then
  echo "Repository directory not found; proceed with build"
  exit 1
fi

# Run git diff
if ! CHANGED_FILES=$(git -C "${REPO_DIR}" diff --no-renames --merge-base "origin/${ghprbTargetBranch}" "${ghprbActualCommit}" --name-only 2>/dev/null); then
  echo "git diff failed; proceed with build"
  exit 1
fi

# Run check_ignored_files.py
if ! echo "${CHANGED_FILES}" | python3 ./scripts/jenkins-scripts/tools/check_ignored_files.py ${IGNORE_ARGS}; \
    status=$?; then
  # Non-ignored files present or check_ignored_files.py errored
  # In either case return the same error code and proceed with build
  exit $status
fi

echo "All changed files match ignored set. Generating dummy reports for publishers..."

# Create dummy JUnit XML for test results publisher
mkdir -p "${WORKSPACE}/build/test_results"
cat << 'EOF' > "${WORKSPACE}/build/test_results/skipped_ci.xml"
<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="0" errors="0" skipped="1">
  <testsuite name="SkippedCI" tests="1" failures="0" errors="0" skipped="1">
    <testcase name="SkippedDueToIgnoredFiles" classname="CI">
      <skipped message="CI build skipped because only ignored/non-code files changed."/>
    </testcase>
  </testsuite>
</testsuites>
EOF

# Create dummy Cppcheck XML for cppcheck publisher
mkdir -p "${WORKSPACE}/build/cppcheck_results"
cat << 'EOF' > "${WORKSPACE}/build/cppcheck_results/skipped_ci.xml"
<?xml version="1.0" encoding="UTF-8"?>
<results version="2">
  <cppcheck version="2.0"/>
  <errors/>
</results>
EOF

# Create dummy ABI report for HTML publisher
mkdir -p "${WORKSPACE}/reports"
cat << 'EOF' > "${WORKSPACE}/reports/compat_report.html"
<!DOCTYPE html>
<html>
<head><title>ABI Report Skipped</title></head>
<body>
  <h1>ABI Report Skipped</h1>
  <p>CI build and ABI check were skipped because only ignored/non-code files were changed in this PR.</p>
</body>
</html>
EOF

echo "Skipping CI"
exit 0
