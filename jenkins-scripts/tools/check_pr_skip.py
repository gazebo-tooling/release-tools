#!/usr/bin/env python3
"""
Check if a PR should skip CI based on ignored file patterns.

Usage:
  python3 check_pr_skip.py <lib_name> [pattern_file1 pattern_file2 ...]

Exit Codes:
  0 - All changed files match ignore patterns (skip CI).
      Dummy test/cppcheck/ABI reports are generated.
  1 - Build required (non-PR build, keyword absent, non-ignored files).
  2 - Usage error (missing lib_name) or check error. Callers should treat non-zero as "build".
"""

import argparse
import os
import shutil
import subprocess
import sys

# Import functions from check_ignored_files in the same directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import check_ignored_files
except ImportError:
    # In case the script is invoked from a different working directory
    # and sys.path[0] resolution requires fallback
    from jenkins_scripts.tools import check_ignored_files


JUNIT_DUMMY_XML = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="0" errors="0" skipped="1">
  <testsuite name="SkippedCI" tests="1" failures="0" errors="0" skipped="1">
    <testcase name="SkippedDueToIgnoredFiles" classname="CI">
      <skipped message="CI build skipped because only ignored/non-code files changed."/>
    </testcase>
  </testsuite>
</testsuites>
"""

CPPCHECK_DUMMY_XML = """<?xml version="1.0" encoding="UTF-8"?>
<results version="2">
  <cppcheck version="2.0"/>
  <errors/>
</results>
"""

ABI_DUMMY_HTML = """<!DOCTYPE html>
<html>
<head><title>ABI Report Skipped</title></head>
<body>
  <h1>ABI Report Skipped</h1>
  <p>CI build and ABI check were skipped because only ignored/non-code files were changed in this PR.</p>
</body>
</html>
"""


def generate_dummy_reports(workspace):
    """Generate dummy XML and HTML reports for publishers when CI is skipped."""
    # Test results
    test_results_dir = os.path.join(workspace, "build", "test_results")
    shutil.rmtree(test_results_dir, ignore_errors=True)
    os.makedirs(test_results_dir, exist_ok=True)
    with open(os.path.join(test_results_dir, "skipped_ci.xml"), "w", encoding="utf-8") as f:
        f.write(JUNIT_DUMMY_XML)

    # Cppcheck results
    cppcheck_results_dir = os.path.join(workspace, "build", "cppcheck_results")
    shutil.rmtree(cppcheck_results_dir, ignore_errors=True)
    os.makedirs(cppcheck_results_dir, exist_ok=True)
    with open(os.path.join(cppcheck_results_dir, "skipped_ci.xml"), "w", encoding="utf-8") as f:
        f.write(CPPCHECK_DUMMY_XML)

    # ABI reports
    reports_dir = os.path.join(workspace, "reports")
    shutil.rmtree(reports_dir, ignore_errors=True)
    os.makedirs(reports_dir, exist_ok=True)
    with open(os.path.join(reports_dir, "compat_report.html"), "w", encoding="utf-8") as f:
        f.write(ABI_DUMMY_HTML)


def main():
    parser = argparse.ArgumentParser(
        description="Check if PR should skip CI based on ignored file patterns.",
        usage="%(prog)s <lib_name> [pattern_file1 pattern_file2 ...]"
    )
    parser.add_argument("lib_name", nargs="?", default=None, help="Name of the library/repo directory")
    parser.add_argument("pattern_files", nargs="*", help="Optional pattern files")

    args = parser.parse_args()

    if not args.lib_name:
        parser.print_usage(sys.stderr)
        sys.exit(2)

    lib_name = args.lib_name
    pattern_files = args.pattern_files

    if not pattern_files:
        default_pattern_file = os.path.join("scripts", "jenkins-scripts", "tools", "gz_ci.ignored")
        pattern_files = [default_pattern_file]

    ghprb_target_branch = os.environ.get("ghprbTargetBranch")
    ghprb_actual_commit = os.environ.get("ghprbActualCommit")

    if not ghprb_target_branch or not ghprb_actual_commit:
        print("Not a PR build or environment variables missing; do not skip build")
        sys.exit(1)

    disable_skipping_keyword = "DISABLE_CI_SKIPPING_FOR_IGNORED_FILES"
    pull_description = os.environ.get("ghprbPullLongDescription", "")
    if disable_skipping_keyword in pull_description:
        print(f"Keyword '{disable_skipping_keyword}' found in PR description; do not skip build")
        sys.exit(1)

    workspace = os.environ.get("WORKSPACE", "")
    repo_dir = os.path.join(workspace, lib_name) if workspace else lib_name

    if not os.path.isdir(repo_dir):
        print("Repository directory not found; proceed with build")
        sys.exit(1)

    # Run git diff
    cmd = [
        "git",
        "-C",
        repo_dir,
        "diff",
        "--no-renames",
        "--merge-base",
        f"origin/{ghprb_target_branch}",
        ghprb_actual_commit,
        "--name-only",
    ]
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        changed_files = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("git diff failed; proceed with build")
        sys.exit(1)

    # Use check_ignored_files functionality
    compiled_patterns = check_ignored_files.load_patterns_from_files(pattern_files)

    if not changed_files:
        print("No files provided to check (treating as all files ignored).")
        non_ignored = []
    else:
        print("Analyzing changed files against ignore patterns:")
        print("  Pattern file(s):")
        for pf in pattern_files:
            print(f"    - {pf}")
        print("  Patterns loaded:")
        for p in compiled_patterns:
            print(f"    - {p.pattern}")
        print("  Files analyzed:")
        for f in changed_files:
            print(f"    - {f}")

        if not compiled_patterns:
            print("Warning: Pattern files are empty or contain no valid patterns.", file=sys.stderr)
            non_ignored = changed_files
        else:
            non_ignored = [f for f in changed_files if not check_ignored_files.is_ignored(f, compiled_patterns)]

    if non_ignored:
        print("The following files do NOT match the ignored set:")
        for f in non_ignored:
            print(f)
        print("Proceeding with build (check_ignored_files.py exit status 1)")
        sys.exit(1)

    print("All files match the ignored set.")
    print("All changed files match ignored set. Generating dummy reports for publishers...")
    generate_dummy_reports(workspace)
    print("Skipping CI")
    sys.exit(0)


if __name__ == "__main__":
    main()
