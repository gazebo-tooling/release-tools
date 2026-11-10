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


if __name__ == "__main__":
    unittest.main()
