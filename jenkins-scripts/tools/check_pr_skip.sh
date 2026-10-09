#!/bin/sh
# Script to check if changed files in a PR branch only match ignored patterns.
# If all files match, exits 0 (signaling to skip build).
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

echo "Skipping CI"
exit 0
