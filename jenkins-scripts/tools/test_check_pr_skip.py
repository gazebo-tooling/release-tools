#!/usr/bin/env python3
"""
Unit tests for check_pr_skip.py
"""

import io
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

# Import check_pr_skip
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_pr_skip


class TestCheckPrSkip(unittest.TestCase):

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.workspace = os.path.join(self.test_dir, "workspace")
        self.repo_dir = os.path.join(self.workspace, "test_repo")
        os.makedirs(self.repo_dir)

        # Create pattern file
        self.pattern_file = os.path.join(self.test_dir, "test.ignored")
        with open(self.pattern_file, "w", encoding="utf-8") as f:
            f.write(r"""
# Ignore markdown and workflows
\.md$
^\.github/
""")

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    @patch("sys.argv", ["check_pr_skip.py"])
    def test_missing_lib_name(self):
        with self.assertRaises(SystemExit) as cm:
            check_pr_skip.main()
        self.assertEqual(cm.exception.code, 2)

    @patch.dict(os.environ, {}, clear=True)
    def test_missing_pr_env_vars(self):
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch("sys.argv", test_args):
            with self.assertRaises(SystemExit) as cm:
                check_pr_skip.main()
            self.assertEqual(cm.exception.code, 1)

    def test_disable_skipping_keyword(self):
        env = {
            "ghprbTargetBranch": "main",
            "ghprbActualCommit": "abcdef123",
            "ghprbPullLongDescription": "This PR has DISABLE_CI_SKIPPING_FOR_IGNORED_FILES in text",
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
                self.assertEqual(cm.exception.code, 1)

    def test_missing_repo_dir(self):
        env = {
            "ghprbTargetBranch": "main",
            "ghprbActualCommit": "abcdef123",
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "nonexistent_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
                self.assertEqual(cm.exception.code, 1)

    @patch("subprocess.run")
    def test_git_diff_failure(self, mock_subprocess):
        mock_subprocess.side_effect = subprocess.CalledProcessError(1, ["git"])
        env = {
            "ghprbTargetBranch": "main",
            "ghprbActualCommit": "abcdef123",
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
                self.assertEqual(cm.exception.code, 1)

    @patch("subprocess.run")
    def test_non_ignored_files_require_build(self, mock_subprocess):
        mock_proc = unittest.mock.Mock()
        mock_proc.stdout = "README.md\nsrc/main.cpp\n"
        mock_subprocess.return_value = mock_proc

        env = {
            "ghprbTargetBranch": "main",
            "ghprbActualCommit": "abcdef123",
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
                self.assertEqual(cm.exception.code, 1)

    @patch("subprocess.run")
    def test_all_ignored_files_skip_ci_and_generate_reports(self, mock_subprocess):
        mock_proc = unittest.mock.Mock()
        mock_proc.stdout = "README.md\n.github/workflows/ci.yml\n"
        mock_subprocess.return_value = mock_proc

        env = {
            "ghprbTargetBranch": "main",
            "ghprbActualCommit": "abcdef123",
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
                self.assertEqual(cm.exception.code, 0)

        # Verify dummy reports were generated
        junit_file = os.path.join(self.workspace, "build", "test_results", "skipped_ci.xml")
        cppcheck_file = os.path.join(self.workspace, "build", "cppcheck_results", "skipped_ci.xml")
        compat_file = os.path.join(self.workspace, "reports", "compat_report.html")

        self.assertTrue(os.path.isfile(junit_file))
        self.assertTrue(os.path.isfile(cppcheck_file))
        self.assertTrue(os.path.isfile(compat_file))

        with open(junit_file, "r", encoding="utf-8") as f:
            self.assertIn("<skipped message=", f.read())
        with open(cppcheck_file, "r", encoding="utf-8") as f:
            self.assertIn("<cppcheck version=", f.read())
        with open(compat_file, "r", encoding="utf-8") as f:
            self.assertIn("ABI Report Skipped", f.read())

    @patch("subprocess.run")
    def test_no_changed_files_skips_ci(self, mock_subprocess):
        mock_proc = unittest.mock.Mock()
        mock_proc.stdout = "\n"
        mock_subprocess.return_value = mock_proc

        env = {
            "ghprbTargetBranch": "main",
            "ghprbActualCommit": "abcdef123",
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
                self.assertEqual(cm.exception.code, 0)


class TestCheckPrSkipWithGit(unittest.TestCase):

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.workspace = os.path.join(self.test_dir, "workspace")
        self.repo_dir = os.path.join(self.workspace, "test_repo")
        os.makedirs(self.repo_dir)

        self.pattern_file = os.path.join(self.test_dir, "test.ignored")
        with open(self.pattern_file, "w", encoding="utf-8") as f:
            f.write(r"""
# Ignore markdown and workflows
\.md$
^\.github/
""")

        # Initialize a real git repo with an origin/main ref
        self._git("init")
        self._git("checkout", "-b", "main")
        self._git("config", "user.name", "Test User")
        self._git("config", "user.email", "test@example.com")

        self._commit_file("src/main.cpp", "int main() { return 0; }\n", "Initial commit")
        self._git("update-ref", "refs/remotes/origin/main", "HEAD")

        # Start a PR branch from main
        self._git("checkout", "-b", "pr-branch")

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def _git(self, *args):
        return subprocess.run(
            ["git", "-C", self.repo_dir, *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        ).stdout.strip()

    def _commit_file(self, rel_path, content, message):
        full_path = os.path.join(self.repo_dir, rel_path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        with open(full_path, "w", encoding="utf-8") as f:
            f.write(content)
        self._git("add", rel_path)
        self._git("commit", "-m", message)

    def _run_check(self, commit_sha=None, target_branch="main"):
        if commit_sha is None:
            commit_sha = self._git("rev-parse", "HEAD")
        env = {
            "ghprbTargetBranch": target_branch,
            "ghprbActualCommit": commit_sha,
            "WORKSPACE": self.workspace,
        }
        test_args = ["check_pr_skip.py", "test_repo", self.pattern_file]
        with patch.dict(os.environ, env, clear=True):
            with patch("sys.argv", test_args):
                with self.assertRaises(SystemExit) as cm:
                    check_pr_skip.main()
        return cm.exception.code

    def test_all_ignored_files_skip_ci(self):
        self._commit_file("README.md", "# Hello\n", "Update README")
        self._commit_file(".github/workflows/ci.yml", "name: CI\n", "Update workflow")

        self.assertEqual(self._run_check(), 0)
        self.assertTrue(
            os.path.isfile(os.path.join(self.workspace, "build", "test_results", "skipped_ci.xml"))
        )

    def test_non_ignored_files_require_build(self):
        self._commit_file("README.md", "# Hello\n", "Update README")
        self._commit_file("src/other.cpp", "void foo() {}\n", "Add cpp file")

        self.assertEqual(self._run_check(), 1)

    def test_merge_base_ignores_new_commits_on_target_branch(self):
        # Advance origin/main with a non-ignored file after pr-branch diverged
        self._git("checkout", "main")
        self._commit_file("src/upstream.cpp", "void upstream() {}\n", "Upstream change")
        self._git("update-ref", "refs/remotes/origin/main", "HEAD")

        # PR branch only modifies an ignored file
        self._git("checkout", "pr-branch")
        self._commit_file("README.md", "# PR change\n", "Update README")

        self.assertEqual(self._run_check(), 0)

    def test_rename_non_ignored_to_ignored_requires_build(self):
        # --no-renames ensures src/main.cpp is still reported as changed
        self._git("mv", "src/main.cpp", "docs.md")
        self._git("commit", "-m", "Rename cpp to md")

        self.assertEqual(self._run_check(), 1)

    def test_empty_commit_skips_ci(self):
        self._git("commit", "--allow-empty", "-m", "Empty commit")

        self.assertEqual(self._run_check(), 0)

    def test_invalid_commit_fails_git_diff_and_requires_build(self):
        self.assertEqual(self._run_check(commit_sha="0000000000000000000000000000000000000000"), 1)


if __name__ == "__main__":
    unittest.main()
