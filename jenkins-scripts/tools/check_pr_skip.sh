#!/bin/sh
# Script to check if changed files in a PR branch only match ignored patterns.
# If all files match, generates dummy test/cppcheck/ABI reports and exits 0 (signaling to skip build).
# Otherwise exits 1 (signaling to proceed with build).

set -e

LIB_NAME="${1:-}"
PATTERN_FILE="${2:-./scripts/jenkins-scripts/tools/gz_ci.ignored}"

if [ -z "${ghprbTargetBranch:-}" ] || [ -z "${ghprbActualCommit:-}" ]; then
  # Not a PR build or environment variables missing; do not skip build
  exit 1
fi

if [ -z "${LIB_NAME}" ]; then
  echo "Usage: $0 <lib_name> [pattern_file]" >&2
  exit 2
fi

REPO_DIR="${WORKSPACE}/${LIB_NAME}"

if [ ! -d "${REPO_DIR}" ]; then
  # Repository directory not found; proceed with build
  exit 1
fi

# Run git diff
if ! CHANGED_FILES=$(git -C "${REPO_DIR}" diff --merge-base "origin/${ghprbTargetBranch}" "${ghprbActualCommit}" --name-only 2>/dev/null); then
  # git diff failed; proceed with build
  exit 1
fi

# Run check_ignored_files.py
if ! echo "${CHANGED_FILES}" | python3 ./scripts/jenkins-scripts/tools/check_ignored_files.py "${PATTERN_FILE}"; then
  # Non-ignored files present; proceed with build
  exit 1
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
