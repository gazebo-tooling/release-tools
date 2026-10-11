#!/usr/bin/env python3
"""
Unit tests for check_ignored_files.py
"""

import unittest
import tempfile
import os
import sys
import io
import re
from unittest.mock import patch

# Import functions from check_ignored_files.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_ignored_files

class TestCheckIgnoredFiles(unittest.TestCase):

    def setUp(self):
        # Create a temporary pattern file for testing
        self.pattern_file = tempfile.NamedTemporaryFile(mode='w+', delete=False, encoding='utf-8')
        self.pattern_file.write(r"""
# This is a comment
^\.github/workflows/

^docs/
\.md$
""")
        self.pattern_file.close()

    def tearDown(self):
        if os.path.exists(self.pattern_file.name):
            os.remove(self.pattern_file.name)

    def test_load_patterns(self):
        patterns = check_ignored_files.load_patterns(self.pattern_file.name)
        self.assertEqual(len(patterns), 3)
        self.assertTrue(patterns[0].search(".github/workflows/ci.yml"))
        self.assertTrue(patterns[1].search("docs/readme.txt"))
        self.assertTrue(patterns[2].search("README.md"))

    def test_load_patterns_file_not_found(self):
        with patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as cm:
                check_ignored_files.load_patterns("non_existent_pattern_file.txt")
            self.assertEqual(cm.exception.code, 2)

    def test_is_ignored(self):
        patterns = [re.compile(r'^\.github/workflows/'), re.compile(r'\.md$')]
        self.assertTrue(check_ignored_files.is_ignored(".github/workflows/ci.yml", patterns))
        self.assertTrue(check_ignored_files.is_ignored("README.md", patterns))
        self.assertFalse(check_ignored_files.is_ignored("src/main.cpp", patterns))
        self.assertFalse(check_ignored_files.is_ignored("docs/index.html", patterns))

    @patch('sys.stdout', new_callable=io.StringIO)
    def test_main_all_ignored_args(self, mock_stdout):
        test_args = ['check_ignored_files.py', self.pattern_file.name, '.github/workflows/ci.yml', 'README.md']
        with patch.object(sys, 'argv', test_args):
            with self.assertRaises(SystemExit) as cm:
                check_ignored_files.main()
            self.assertEqual(cm.exception.code, 0)
            self.assertIn("All files match the ignored set.", mock_stdout.getvalue())

    @patch('sys.stdout', new_callable=io.StringIO)
    def test_main_non_ignored_args(self, mock_stdout):
        test_args = ['check_ignored_files.py', self.pattern_file.name, '.github/workflows/ci.yml', 'src/main.cpp']
        with patch.object(sys, 'argv', test_args):
            with self.assertRaises(SystemExit) as cm:
                check_ignored_files.main()
            self.assertEqual(cm.exception.code, 1)
            output = mock_stdout.getvalue()
            self.assertIn("The following files do NOT match the ignored set:", output)
            self.assertIn("src/main.cpp", output)

    @patch('sys.stdout', new_callable=io.StringIO)
    def test_main_stdin_input(self, mock_stdout):
        test_args = ['check_ignored_files.py', self.pattern_file.name]
        stdin_data = ".github/workflows/ci.yml\ndocs/guide.txt\n"
        with patch.object(sys, 'argv', test_args):
            with patch('sys.stdin', io.StringIO(stdin_data)):
                with patch('sys.stdin.isatty', return_value=False):
                    with self.assertRaises(SystemExit) as cm:
                        check_ignored_files.main()
                    self.assertEqual(cm.exception.code, 0)
                    self.assertIn("All files match the ignored set.", mock_stdout.getvalue())

    def test_main_missing_args(self):
        test_args = ['check_ignored_files.py']
        with patch.object(sys, 'argv', test_args):
            with patch('sys.stderr', new_callable=io.StringIO):
                with self.assertRaises(SystemExit) as cm:
                    check_ignored_files.main()
                self.assertEqual(cm.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
