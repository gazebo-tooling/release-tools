#!/usr/bin/env python3
# Unit tests for the checkout inference and checks in release.py
#   python3 -m unittest -v test_releasepy

import contextlib
import io
import os
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace

import release

RELEASE_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))


def git(cwd, *cmd):
    subprocess.run(['git', *cmd], cwd=cwd, check=True, capture_output=True)


def write(path, content):
    with open(path, 'w') as f:
        f.write(content)


CMAKE_STABLE = """cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(gz-math8 VERSION 8.4.0)
find_package(gz-cmake4 REQUIRED)
gz_configure_project(VERSION_SUFFIX)
"""

CMAKE_MAIN_PRE = """cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(gz-math VERSION 10.0.0)
find_package(gz-cmake REQUIRED)
gz_configure_project(
  REPLACE_INCLUDE_PATH gz/math
  VERSION_SUFFIX pre1)
"""

CHANGELOG = """## Gazebo Math 8.x

### Gazebo Math 8.4.0 (2026-08-25)

1. Fix something
    * [Pull request #123](https://github.com/gazebosim/gz-math/pull/123)

### Gazebo Math 8.3.0 (2026-04-02)
"""


class TestCMakeInference(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cmake = os.path.join(self.tmp.name, 'CMakeLists.txt')

    def tearDown(self):
        self.tmp.cleanup()

    def test_stable_branch_project(self):
        write(self.cmake, CMAKE_STABLE)
        self.assertEqual(release.get_project_from_cmake(self.cmake),
                         ('gz-math8', '8.4.0'))

    def test_main_prerelease_project(self):
        write(self.cmake, CMAKE_MAIN_PRE)
        self.assertEqual(release.get_project_from_cmake(self.cmake),
                         ('gz-math', '10.0.0~pre1'))
        self.assertEqual(release.get_version_from_cmake(self.cmake),
                         '10.0.0~pre1')

    def test_sdformat_project_with_space(self):
        write(self.cmake, "project (sdformat14 VERSION 14.9.0)\n")
        self.assertEqual(release.get_project_from_cmake(self.cmake),
                         ('sdformat14', '14.9.0'))

    def test_missing_cmake(self):
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit):
                release.get_project_from_cmake(self.cmake)

    def test_package_names(self):
        f = release.get_package_from_cmake_project
        self.assertEqual(f('gz-math8', '8.4.0'), 'gz-math8')
        self.assertEqual(f('gz-math', '10.0.0~pre1'), 'gz-math10')
        self.assertEqual(f('sdformat', '17.0.0~pre1'), 'sdformat17')
        self.assertEqual(f('ignition-math6', '6.15.1'), 'ign-math6')
        self.assertEqual(f('ignition-gazebo6', '6.17.0'), 'ign-gazebo6')


class TestInferFromCheckout(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cwd = os.getcwd()
        os.chdir(self.tmp.name)
        write('CMakeLists.txt', CMAKE_STABLE)

    def tearDown(self):
        os.chdir(self.cwd)
        self.tmp.cleanup()

    def args(self, package=None, version=None):
        return SimpleNamespace(package=package, version=version)

    def test_infer_both(self):
        args = self.args()
        release.infer_from_checkout(args)
        self.assertEqual((args.package, args.version), ('gz-math8', '8.4.0'))
        self.assertEqual(args.inferred, ['package', 'version'])

    def test_infer_version_only(self):
        args = self.args('gz-math8')
        release.infer_from_checkout(args)
        self.assertEqual(args.version, '8.4.0')
        self.assertEqual(args.inferred, ['version'])

    def test_package_mismatch(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            with self.assertRaises(SystemExit):
                release.infer_from_checkout(self.args('gz-math7'))
        self.assertIn('CMakeLists.txt says package gz-math8', out.getvalue())

    def test_explicit_args_do_not_read_cmake(self):
        os.remove('CMakeLists.txt')
        args = self.args('gz-math8', '8.4.0')
        release.infer_from_checkout(args)
        self.assertEqual(args.inferred, [])


class TestReleaseVersionInference(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def changelog(self, subdir, line):
        os.makedirs(os.path.join(self.repo, subdir), exist_ok=True)
        write(os.path.join(self.repo, subdir, 'changelog'), line + '\n')

    def infer(self, release_version=None, version='8.4.0'):
        args = SimpleNamespace(version=version,
                               release_version=release_version,
                               inferred=[])
        release.infer_release_version(args, self.repo)
        return args

    def test_revision_from_changelogs(self):
        self.changelog('ubuntu/debian', 'gz-math8 (8.4.0-2~jammy) jammy; urgency=low')
        self.changelog('noble/debian', 'gz-math8 (8.4.0-2~noble) noble; urgency=low')
        args = self.infer()
        self.assertEqual(args.release_version, '2')
        self.assertEqual(args.inferred, ['release version'])

    def test_explicit_revision_wins(self):
        self.changelog('ubuntu/debian', 'gz-math8 (8.4.0-2~jammy) jammy; urgency=low')
        self.assertEqual(self.infer('3').release_version, '3')

    def test_stale_release_repo_defaults_to_1(self):
        self.changelog('ubuntu/debian', 'gz-math8 (8.3.0-1~jammy) jammy; urgency=low')
        args = self.infer()
        self.assertEqual(args.release_version, '1')
        self.assertEqual(args.inferred, [])

    def test_disagreeing_revisions_default_to_1(self):
        self.changelog('ubuntu/debian', 'gz-math8 (8.4.0-2~jammy) jammy; urgency=low')
        self.changelog('noble/debian', 'gz-math8 (8.4.0-1~noble) noble; urgency=low')
        self.assertEqual(self.infer().release_version, '1')


class TestCheckoutChecks(unittest.TestCase):
    """Run the checks on a clone of a local bare 'origin' repository"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cwd = os.getcwd()
        self.argv0 = sys.argv[0]
        # get_expected_branches locates gz-collections.yaml from argv[0]
        sys.argv[0] = os.path.join(RELEASE_TOOLS_DIR, 'release.py')

        self.origin = os.path.join(self.tmp.name, 'origin.git')
        self.src = os.path.join(self.tmp.name, 'src')
        git(self.tmp.name, 'init', '-q', '--bare', '-b', 'gz-math8', self.origin)
        git(self.tmp.name, 'clone', '-q', self.origin, self.src)
        git(self.src, 'config', 'user.email', 'test@test.foo')
        git(self.src, 'config', 'user.name', 'Test')
        git(self.src, 'checkout', '-q', '-b', 'gz-math8')
        write(os.path.join(self.src, 'CMakeLists.txt'), CMAKE_STABLE)
        write(os.path.join(self.src, 'Changelog.md'), CHANGELOG)
        git(self.src, 'add', '.')
        git(self.src, 'commit', '-q', '-m', 'Prepare for 8.4.0')
        git(self.src, 'push', '-q', 'origin', 'gz-math8')
        os.chdir(self.src)

    def tearDown(self):
        os.chdir(self.cwd)
        sys.argv[0] = self.argv0
        self.tmp.cleanup()

    def args(self, package='gz-math8', version='8.4.0'):
        return SimpleNamespace(package=package, package_alias=package,
                               version=version)

    def check(self, args=None):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            try:
                release.checkout_checks(args or self.args())
            except SystemExit:
                return False, out.getvalue()
        return True, out.getvalue()

    def assertFailsWith(self, message, args=None):
        ok, out = self.check(args)
        self.assertFalse(ok, out)
        self.assertIn(message, out)

    def test_all_good(self):
        ok, out = self.check()
        self.assertTrue(ok, out)
        self.assertIn('Branch gz-math8 is the release branch', out)
        self.assertIn('Tag gz-math8_8.4.0 does not exist', out)

    def test_dirty_tree(self):
        write('Changelog.md', CHANGELOG + 'local edit\n')
        self.assertFailsWith('uncommitted or untracked changes')

    def test_untracked_file(self):
        write('stray.txt', 'x')
        self.assertFailsWith('uncommitted or untracked changes')

    def test_detached_head(self):
        git(self.src, 'checkout', '-q', '--detach')
        self.assertFailsWith('detached HEAD')

    def test_local_commit_not_pushed(self):
        git(self.src, 'commit', '-q', '--allow-empty', '-m', 'local')
        self.assertFailsWith('1 commit(s) ahead and 0 behind origin/gz-math8')

    def test_stale_checkout(self):
        other = os.path.join(self.tmp.name, 'other')
        git(self.tmp.name, 'clone', '-q', '-b', 'gz-math8', self.origin, other)
        git(other, '-c', 'user.email=a@b.c', '-c', 'user.name=A',
            'commit', '-q', '--allow-empty', '-m', 'upstream')
        git(other, 'push', '-q', 'origin', 'gz-math8')
        self.assertFailsWith('0 commit(s) ahead and 1 behind origin/gz-math8')

    def test_wrong_branch(self):
        git(self.src, 'checkout', '-q', '-b', 'gz-math7')
        git(self.src, 'push', '-q', 'origin', 'gz-math7')
        self.assertFailsWith('Expected gz-math8')

    def test_main_is_a_warning(self):
        git(self.src, 'checkout', '-q', '-b', 'main')
        git(self.src, 'push', '-q', 'origin', 'main')
        ok, out = self.check()
        self.assertTrue(ok, out)
        self.assertIn('WARNING releasing gz-math8 from main', out)

    def test_missing_changelog_entry(self):
        self.assertFailsWith('Changelog.md has no entry for 8.4.1',
                             self.args(version='8.4.1'))

    def test_changelog_does_not_match_substrings(self):
        # 8.4.0 is in the changelog, 18.4.0 and 8.4.00 are not
        self.assertFailsWith('no entry for 18.4.0', self.args(version='18.4.0'))

    def test_prerelease_changelog_entry(self):
        ok, out = self.check(self.args(version='8.4.0~pre1'))
        self.assertTrue(ok, out)

    def test_tag_exists_in_origin(self):
        git(self.src, 'tag', 'gz-math8_8.4.0')
        git(self.src, 'push', '-q', 'origin', 'gz-math8_8.4.0')
        git(self.src, 'tag', '-d', 'gz-math8_8.4.0')
        self.assertFailsWith('Tag gz-math8_8.4.0 already exists in origin')


if __name__ == '__main__':
    unittest.main()
