#!/usr/bin/env python3

from __future__ import print_function
from argparse import RawTextHelpFormatter
from configparser import ConfigParser
from typing import Tuple
from urllib3.exceptions import RequestError
from urllib3.util import make_headers
import re
import subprocess
import sys
import tempfile
import os
import urllib.parse
import urllib3
import argparse
import shutil
import venv

USAGE = 'release.py [<package> <version>]'
try:
    JENKINS_URL = os.environ['JENKINS_URL']
except KeyError:
    JENKINS_URL = 'https://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-debbuilder'
GENERIC_BREW_PULLREQUEST_JOB = 'generic-release-homebrew_pull_request_updater'

LINUX_DISTROS = ['ubuntu', 'debian']
SUPPORTED_ARCHS = ['amd64', 'arm64']
RELEASEPY_NO_ARCH_PREFIX = '.releasepy_NO_ARCH_'
ROS_VENDOR = {'harmonic': ['jazzy'],
              'ionic': ['kilted'],
              'jetty': ['lyrical'],
              'm': ['rolling']
              }

OSRF_REPOS_SUPPORTED = "stable prerelease nightly testing none"

DRY_RUN = False
NIGHTLY = False
PRERELEASE = False

IGNORE_DRY_RUN = True


class ErrorNoPermsRepo(Exception):
    pass


class ErrorNoUsernameSupplied(Exception):
    pass


class ErrorURLNotFound404(Exception):
    pass


class ErrorNoOutput(Exception):
    pass


class ErrorAlreadyExists(Exception):
    pass


def error(msg):
    print("\n !! " + msg + "\n")
    sys.exit(1)


def print_success(msg):
    print(" + OK " + msg)


def print_only_dbg(msg):
    if '_RELEASEPY_DEBUG' in os.environ:
        print(msg)


# Remove the last character if it is a number.
# That should leave just the package name instead of packageVersion
# I.E gazebo5 -> gazebo
def get_canonical_package_name(pkg_name):
    return pkg_name.rstrip('1234567890')


def replace_ignition_gz(pkg_name):
    return pkg_name \
        .replace('ignition-gazebo', 'gz-sim') \
        .replace('ignition-', 'gz-')


def github_repo_exists(url):
    try:
        check_call(['git', 'ls-remote', '-q', '--exit-code', url], IGNORE_DRY_RUN)
    except (ErrorURLNotFound404, ErrorNoOutput):
        return False
    except Exception as e:
        error("Unexpected problem checking for git repo: " + str(e))
    return True


def exists_main_branch(github_url):
    check_main_cmd = ['git', 'ls-remote', '--exit-code', '--heads', github_url, 'main']
    try:
        if (check_call(check_main_cmd, IGNORE_DRY_RUN)):
            return True
    except Exception:
        return False


def parse_args(argv):
    global DRY_RUN
    global NIGHTLY
    global PRERELEASE

    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter,
    description="""
Script to handle the release process for the Gazebo devs.
Examples:
A) Generate source: local repository tag + call source job:
   $ release.py <package> <version>
   (auto calculate source-repo-uri from local directory)

B) Call builders: reuse existing tarball version + call build jobs:
   $ release.py --source-tarball-uri <URL> <package> <version>
   (no call to source job, directly build jobs with tarball URL)
   # Optionally include SHA256 checksum for verification
   $ release.py --source-tarball-uri <URL> --source-tarball-sha256 <SHA256> <package> <version>

C) Nightly builds (linux)
   $ release.py --source-repo-existing-ref <git_branch> --upload-to-repo nightly <package> nightly

D) Infer the release from the source checkout:
   $ release.py
   (package and version from CMakeLists.txt, release version from the -release
    repo; checks the checkout, prints the release plan and asks for confirmation)
 """)
    parser.add_argument('package', nargs='?', default=None,
                        help='which package to release (default: from CMakeLists.txt)')
    parser.add_argument('version', nargs='?', default=None,
                        help='which version to release (default: from CMakeLists.txt)')
    parser.add_argument('deprecated_jenkins_token',
                        default=None,
                        nargs="?",
                        help=argparse.SUPPRESS)
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    parser.add_argument('--auth', dest='auth_input_arg',
                        default="",
                        help='Explicit jenkins user:token string overriding the jenkins.ini credentials file.')
    parser.add_argument('-a', '--package-alias', dest='package_alias',
                        default=None,
                        help='different name that we are releasing under')
    parser.add_argument('-b', '--release-repo-branch', dest='release_repo_branch',
                        default='master',
                        help='which version of the accompanying -release repo to use (if not default)')
    parser.add_argument('-r', '--release-version', dest='release_version',
                        default=None,
                        help='Release version suffix; usually 1 (e.g., 1')
    parser.add_argument('--no-sanity-checks', dest='no_sanity_checks', action='store_true', default=False,
                        help='no-sanity-checks; i.e. skip sanity checks commands')
    parser.add_argument('--source-repo-uri',
                        dest='source_repo_uri',
                        default=None,
                        help='Indicate the repository URL to grab the source from (overriding the heristics to calculate it from the local directory)')  # NOQA
    parser.add_argument('--source-repo-existing-ref',
                        dest='source_repo_ref',
                        default=None,
                        help='Optionally, when using --source-repo-uri, indicate the Git reference (branch|tag) to grab the release sources from.\
                              If used: avoid to tag the local repository. If not used: tag the local repository with <version> and use it as ref')  # NOQA
    parser.add_argument('--source-tarball-uri',
                        dest='source_tarball_uri', default=None,
                        help='Indicate the URL of the sources to grab the release sources from.')  # NOQA
    parser.add_argument('--source-tarball-sha256',
                        dest='source_tarball_sha256', default=None,
                        help='SHA256 checksum of the tarball specified in --source-tarball-uri.')  # NOQA
    parser.add_argument('--upload-to-repo', dest='upload_to_repository', default=None,
                        help='OSRF repo to upload: stable | prerelease | nightly\n'
                             '(default: prerelease if the version has ~pre, stable otherwise)')
    parser.add_argument('--extra-osrf-repo', dest='extra_repo', default="",
                        help='extra OSRF repository to use in the build')
    parser.add_argument('--nightly-src-branch', dest='nightly_branch', default="main",
                        help='branch in the source code repository to build the nightly from')
    parser.add_argument('--only-bump-revision-linux', dest='bump_rev_linux_only',
                        action='store_true', default=False,
                        help='Bump only revision number. Do not upload new tarball.')
    parser.add_argument('--only-bump-ros-vendor-package', dest='bump_ros_vendor_only',
                        action='store_true', default=False,
                        help='Only process the ROS vendor package (if any).')
    parser.add_argument('-y', '--yes', dest='yes', action='store_true', default=False,
                        help='Do not ask for confirmation before releasing when package\n'
                             'or version were inferred from the checkout')

    args = parser.parse_args()
    DRY_RUN = args.dry_run

    return args


def finalize_args(args):
    global NIGHTLY
    global PRERELEASE

    args.package_alias = args.package
    if args.package.startswith('ign-'):
        args.package_alias = args.package.replace('ign-', 'ignition-')

    if not args.upload_to_repository:
        args.upload_to_repository = \
            'prerelease' if '~pre' in args.version else 'stable'
        args.inferred.append('upload repo')

    if args.upload_to_repository == 'nightly':
        NIGHTLY = True
    if args.upload_to_repository == 'prerelease':
        PRERELEASE = True

#
# BEGIN: Credentials code copied from ros_buildfarm
#
def get_credentials(jenkins_url=None):
    config = ConfigParser()
    config_file = get_credential_path()
    if not os.path.exists(config_file):
        print("Could not find credential file '%s'" % config_file,
              file=sys.stderr)
        return None, None

    config.read(config_file)
    section_name = None
    if jenkins_url is not None and jenkins_url in config:
        section_name = jenkins_url
    if section_name is None and 'DEFAULT' in config:
        section_name = 'DEFAULT'

    if section_name is None or 'username' not in config[section_name] or \
            'password' not in config[section_name]:
        print(
            "Could not find credentials for '%s' in file '%s'" %
            (jenkins_url, config_file), file=sys.stderr)
        return None, None
    return config[section_name]['username'], config[section_name]['password']


def get_credential_path():
    return os.path.join(
        os.path.expanduser('~'), get_relative_credential_path())


def get_relative_credential_path():
    return os.path.join('.buildfarm', 'jenkins.ini')

#
# END: Credentials code copied from ros_buildfarm
#

def get_release_repository_info(package):
    github_url = "https://github.com/gazebo-release/" + package + "-release"
    if (github_repo_exists(github_url)):
        return 'git', github_url

    error("release repository not found in github.com/gazebo-release")


def download_release_repository(package, release_branch):
    try:
        return os.environ['_RELEASEPY_TEST_RELEASE_REPO'], 'main'
    except KeyError:
        vcs, url = get_release_repository_info(package)
        release_tmp_dir = tempfile.mkdtemp()

        if vcs == "git" and release_branch == "default":
            release_branch = "master"

        # If main branch exists, prefer it over master
        if release_branch == "master":
            if exists_main_branch(url):
                release_branch = 'main'

        cmd = [vcs, "clone", "-b", release_branch, url, release_tmp_dir]
        check_call(cmd, IGNORE_DRY_RUN)
        return release_tmp_dir, release_branch


def sanity_package_name_underscore(package, package_alias):
    # Alias is never empty. It hosts a exect copy of package if not provided
    if '_' in package_alias and package_alias != package:
        error("Found an underscore in package_alias. It will conflict with debian package names. May be fixed changing the underscore for a dash.")

    if '_' in package and package_alias == package:
        error("Found an underscore in package name without providing a package alias (-a <alias>). You probably want to match the package name in the debian changelog")

    print_success("No underscore in package name")


def sanity_package_name(repo_dir, package, package_alias):
    expected_name = package

    if package_alias:
        expected_name = package_alias

    # Use igntiion for Citadel and Fortress, gz for Garden and beyond
    gz_name = expected_name.replace("ignition", "gz")
    gz_name = gz_name.replace("gazebo", "sim")
    # For rotary packages (e.g. gz-rotary-cmake), the release repo uses the
    # base name without the '-rotary-' infix (e.g. gz-cmake)
    if expected_name == 'gz-rotary-sdformat':
        # sdformat release repositories use the un-prefixed source package name.
        gz_name = 'sdformat'
    else:
        gz_name = expected_name.replace('-rotary-', '-')

    cmd = ["find", repo_dir, "-name", "changelog", "-exec", "head", "-n", "1", "{}", ";"]
    out, _ = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.decode().split('\n'):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[0] != expected_name and line.partition(' ')[0] != gz_name:
            error("Error in changelog package name or alias: " + line)

    cmd = ["find", repo_dir, "-name", "control", "-exec", "grep", "-H", "Source:", "{}", ";"]
    out, _ = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.decode().split('\n'):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[2] != expected_name and line.partition(' ')[2] != gz_name:
            error("Error in source package. File:  " + line.partition(' ')[1] + ". Got " + line.partition(' ')[2] + " expected " + expected_name + " or " + gz_name)

    print_success("Package names in changelog and control")


def get_release_repo_changelog_versions(repo_dir) -> list:
    # (version, revision, full version) of the top entry of each changelog
    cmd = ["find", repo_dir, "-name", "changelog", "-exec", "head", "-n", "1", "{}", ";"]
    out, _ = check_call(cmd, IGNORE_DRY_RUN)
    versions = []
    for line in out.decode().split('\n'):
        if not line:
            continue
        # return full version in brackets
        full_version = line.split(' ')[1]
        # get only version (not release) in brackets
        c_version = full_version[full_version.find("(")+1:full_version.find("-")]
        c_revision = full_version[full_version.find("-")+1:full_version.rfind("~")]
        versions.append((c_version, c_revision, full_version))
    return versions


def infer_release_version(args, repo_dir):
    if args.release_version:
        return
    # Use the revision of the -release repo changelogs when they all agree on
    # it for this version. Otherwise default to 1 and let the sanity checks
    # report the mismatch.
    revisions = {revision for c_version, revision, _ in
                 get_release_repo_changelog_versions(repo_dir)
                 if c_version == args.version}
    if len(revisions) == 1:
        args.release_version = revisions.pop()
        args.inferred.append('release version')
    else:
        args.release_version = '1'


def sanity_package_version(repo_dir, version, release_version):
    for c_version, c_revision, full_version in \
            get_release_repo_changelog_versions(repo_dir):
        if c_version != version:
            error("Error in package version. Repo version: " + c_version + " Provided version: " + version)

        if c_revision != release_version:
            error("Error in package release version. Expected " + release_version + " in line " + full_version)

    print_success("Package versions in changelog")
    print_success("Package release versions in changelog")


def sanity_check_sdformat_versions(package, version):
    if package == 'sdformat':
        if int(version[0]) > 1:
            error("Error is sdformat version. Please use 'sdformatX' (with version number) for package name")
        else:
            return

    print_success("sdformat version in proper sdformat package")

def get_project_from_cmake(cmake_file="CMakeLists.txt") -> Tuple[str, str]:
    project_regex = re.compile(
        r"project\s*\(\s*([a-z0-9-_]*)\s*VERSION\s*([0-9.]*)", re.MULTILINE
    )
    # Note the re.DOTALL is used to match any newlines and arguments to
    # gz_configure_project before VERSION_SUFFIX
    suffix_regex = re.compile(
        r"(?:gz|ign)_configure_project\s*\(.*VERSION_SUFFIX\s*(pre\d+)",
        re.MULTILINE | re.DOTALL,
    )
    try:
        with open(cmake_file) as f:
            content = f.read()
    except FileNotFoundError as e:
        print(e)
        error("Could not find CMakeLists file. Are you sure you're in the source directory?")

    project_match = re.search(project_regex, content)
    if not project_match or not project_match.group(2):
        error("Error parsing version from CMakeLists.txt file")
    cmake_version = project_match.group(2)
    suffix_match = re.search(suffix_regex, content)
    if suffix_match:
        cmake_version = f"{cmake_version}~{suffix_match.group(1)}"
    return project_match.group(1), cmake_version


def get_version_from_cmake(cmake_file="CMakeLists.txt"):
    return get_project_from_cmake(cmake_file)[1]


def get_package_from_cmake_project(project_name, version):
    # Stable branches use the major version in the project name
    # (gz-math7, sdformat14, ignition-math6) but main does not (gz-math),
    # the package name always carries it.
    package = project_name
    if package.startswith('ignition-'):
        package = package.replace('ignition-', 'ign-', 1)
    if not package[-1].isdigit():
        package += version.split('.')[0]
    return package


def infer_from_checkout(args):
    args.inferred = []
    if args.package and args.version:
        return

    project_name, cmake_version = get_project_from_cmake()
    if not project_name:
        error("Could not read the project name from CMakeLists.txt. "
              "Please pass <package> <version> explicitly")
    cmake_package = get_package_from_cmake_project(project_name, cmake_version)

    for arg_name, cmake_value in (('package', cmake_package),
                                  ('version', cmake_version)):
        value = getattr(args, arg_name)
        if not value:
            setattr(args, arg_name, cmake_value)
            args.inferred.append(arg_name)
        elif value != cmake_value:
            error(f"CMakeLists.txt says {arg_name} {cmake_value}, "
                  f"command line says {value}")


def sanity_check_cmake_version(package, version):
    # These two packages do not follow the same formatting in their CMakeLists files.
    # Since they are old versions, we'll simply not support them.
    if package in ["ign-tools", "sdformat9"]:
        print(f" + NOTE Sanity checking is not supported for {package}")
        return

    cmake_version = get_version_from_cmake()
    if cmake_version != version:
        error(f"Error in package version. CMakeLists version: {cmake_version}, provided version: {version}")
    else:
        print_success("Package version in CMakeLists")

def sanity_check_repo_name(repo_name):
    if repo_name in OSRF_REPOS_SUPPORTED:
        return

    error(f"Upload repo value: {repo_name} is not valid. \
            Supported values are: {OSRF_REPOS_SUPPORTED}")


def sanity_project_package_in_stable(version, repo_name):
    if repo_name != 'stable':
        return

    if '+' in version:
        error("Detected stable repo upload using project versioning scheme (include '+' in the version)")

    if '~pre' in version:
        error("Detected stable repo upload using project versioning scheme (include '~pre' in the version)")

    return


def sanity_check_source_repo_uri(source_repo_uri):
    # Check if the scheme is "https" and the path ends with ".git"
    parsed_uri = urllib.parse.urlparse(source_repo_uri)
    if not parsed_uri.scheme == "https" or \
       not parsed_uri.path.endswith(".git"):
        error("--source-repo-uri parameter should start with https:// and end with .git")
    # Needs to be fully in sync for SOURCE_REPO_URI values in
    # OSRFSourceCreation
    # https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/dsl/gazebo_libs.dsl#L513
    if parsed_uri.netloc.startswith("github.org"):
        error("--source-repo-uri needs github.com instead of github.org")


def sanity_check_bump_linux(source_tarball_uri):
    if (not source_tarball_uri):
        error('--only-bump-revision-linux needs --source-tarball-uri argument'
              'to call builders and not source generation')


def sanity_checks(args, repo_dir):
    print("Safety checks:")
    sanity_package_name_underscore(args.package, args.package_alias)
    sanity_package_name(repo_dir, args.package, args.package_alias)
    sanity_check_repo_name(args.upload_to_repository)

    if (args.bump_rev_linux_only):
        sanity_check_bump_linux(args.source_tarball_uri)

    if args.source_repo_uri:
        sanity_check_source_repo_uri(args.source_repo_uri)

    if not NIGHTLY:
        sanity_package_version(repo_dir, args.version, str(args.release_version))
        sanity_check_sdformat_versions(args.package, args.version)
        if not (args.bump_rev_linux_only or args.source_tarball_uri):
            sanity_check_cmake_version(args.package, args.version)
        sanity_project_package_in_stable(args.version, args.upload_to_repository)

    check_credentials(args.auth_input_arg)
    print_success("Jenkins credentials are good")
    shutil.rmtree(repo_dir)


def get_exclusion_arches(files):
    r = []
    for f in files:
        if f.startswith(RELEASEPY_NO_ARCH_PREFIX):
            arch = f.replace(RELEASEPY_NO_ARCH_PREFIX, '').lower()
            r.append(arch)

    return r


def discover_distros(repo_dir):
    if not os.path.isdir(repo_dir):
        return None

    _, subdirs, files = os.walk(repo_dir).__next__()
    repo_arch_exclusion = get_exclusion_arches(files)

    if '.git' in subdirs:
        subdirs.remove('.git')
    # remove ubuntu (common stuff) and debian (supported distro at top level)
    if 'ubuntu' in subdirs:
        subdirs.remove('ubuntu')
    if 'debian' in subdirs:
        subdirs.remove('debian')
    # Some releasing methods use patches/ in root
    if 'patches' in subdirs:
        subdirs.remove('patches')

    if not subdirs:
        error('Can not find distributions directories in the -release repo')

    distro_arch_list = {}
    for d in subdirs:
        files = os.walk(repo_dir + '/' + d).__next__()[2]
        distro_arch_exclusion = get_exclusion_arches(files)
        excluded_arches = distro_arch_exclusion + repo_arch_exclusion
        arches_supported = [x for x in SUPPORTED_ARCHS if x not in excluded_arches]
        distro_arch_list[d] = arches_supported

    print('Linux distributions in the -release repository:')
    for distro in distro_arch_list:
        print(f" + {distro}  {*distro_arch_list[distro],}")

    return distro_arch_list


def check_call(cmd, ignore_dry_run=False, cwd=None):
    if DRY_RUN and not ignore_dry_run:
        print_only_dbg('Dry-run running:\n  %s\n' % (' '.join(cmd)))
        return b'', b''
    else:
        print_only_dbg('Running:\n  %s' % (' '.join(cmd)))
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=cwd)
        out, err = po.communicate()
        if po.returncode != 0:
            # bitbucket for the first one, github for the second
            if (b"404" in err) or (b"Repository not found" in err):
                raise ErrorURLNotFound404()
            if b"Permission denied" in out:
                raise ErrorNoPermsRepo()
            if b"abort: no username supplied" in err:
                raise ErrorNoUsernameSupplied()
            if b"already exists:" in err:
                raise ErrorAlreadyExists()
            if not out and not err:
                # assume that call is only for getting return code
                raise ErrorNoOutput()

            # Unkown exception
            print('Error running command (%s).' % (' '.join(cmd)))
            print('stdout: %s' % (out.decode()))
            print('stderr: %s' % (err.decode()))
            raise Exception('subprocess call failed')
        return out, err


def get_tag_name(args):
    # tilde is not a valid character in git
    return '%s_%s' % (args.package_alias, args.version.replace('~', '-'))


def tags_local_repo(args):
    # Source mode without an existing ref tags the local HEAD
    return not (NIGHTLY or args.source_tarball_uri or args.source_repo_ref)


def git_output(*cmd):
    try:
        out, _ = check_call(['git', *cmd], IGNORE_DRY_RUN)
    except ErrorNoOutput:
        return ''
    except Exception:
        error(f"Failed to run 'git {' '.join(cmd)}' in the local checkout")
    return out.decode().strip()


def get_changelog_versions(changelog_file='Changelog.md') -> list:
    versions = []
    with open(changelog_file) as f:
        for line in f:
            if line.startswith('#'):
                versions += re.findall(r'(?<![\d.])\d+\.\d+\.\d+(?![\d.])', line)
    return versions


def checkout_checks(args):
    # The release tags the local HEAD: check that it is the reviewed
    # version bump in the expected branch.
    print("Checkout checks:")
    if git_output('status', '--porcelain'):
        error("The working tree has uncommitted or untracked changes. "
              "Release from a clean checkout")
    print_success("Working tree is clean")

    branch = git_output('rev-parse', '--abbrev-ref', 'HEAD')
    if branch == 'HEAD':
        error("The checkout is in detached HEAD. Checkout the branch to release")
    git_output('fetch', '--tags', 'origin', branch)
    counts = git_output('rev-list', '--left-right', '--count',
                        f'HEAD...origin/{branch}').split()
    if counts != ['0', '0']:
        ahead, behind = counts
        error(f"Local {branch} is {ahead} commit(s) ahead and {behind} "
              f"behind origin/{branch}. Pull or push first")
    print_success(f"Local {branch} is up to date with origin/{branch}")

    expected_branches = get_expected_branches(args)
    if not expected_branches:
        print(f" ~ WARNING no branch for {args.package} in gz-collections.yaml,"
              " not checking the branch")
    elif branch in expected_branches:
        print_success(f"Branch {branch} is the release branch for {args.package}")
    elif branch == 'main':
        print(f" ~ WARNING releasing {args.package} from main instead of "
              f"{' or '.join(expected_branches)}. Expected only for the first"
              " release of a major version")
    else:
        error(f"Branch {branch} is not the release branch for {args.package}. "
              f"Expected {' or '.join(expected_branches)}")

    if not os.path.isfile('Changelog.md'):
        print(" ~ WARNING no Changelog.md found, not checking the changelog")
    elif args.version.split('~')[0] not in get_changelog_versions():
        error(f"Changelog.md has no entry for {args.version}. "
              "Was the version bump PR merged?")
    else:
        print_success(f"Changelog.md has an entry for {args.version}")

    tag = get_tag_name(args)
    if git_output('ls-remote', '--tags', 'origin', f'refs/tags/{tag}'):
        error(f"Tag {tag} already exists in origin")
    print_success(f"Tag {tag} does not exist in origin")


def tag_repo(args):
    try:
        tag = get_tag_name(args)
        check_call(['git', 'tag', '-f', tag])
        check_call(['git', 'push', 'origin', 'tag', tag])
    except ErrorNoPermsRepo:
        print('The Git server reports problems with permissions')
        print('The branch could be blocked by configuration if you do not have')
        print('rights to push code in default branch.')
        sys.exit(1)
    except ErrorNoUsernameSupplied:
        print('git tag could not be committed because you have not configured')
        print('your username. Use "git config --username" to set your username.')
        sys.exit(1)
    except Exception:
        print('There was a problem with pushing tags to the git repository')
        print('Do you have write perms in the repository?')
        sys.exit(1)

    return tag


def generate_source_repository_uri(args):
    org_repo = f"gazebosim/{get_canonical_package_name(args.package_alias)}"
    out, err = check_call(['git', 'ls-remote', '--get-url', 'origin'],
                          IGNORE_DRY_RUN)
    if err:
        print(f"An error happened running git ls-remote: {err}")
        sys.exit(1)

    git_remote = out.decode().split('\n')[0]
    if org_repo not in git_remote:
        # Handle the special case for citadel ignition repositories
        if replace_ignition_gz(org_repo) not in git_remote:
            print(f""" !! Automatic calculation of the source repository URI\
                  failed with different information:\
                  \n   * git remote origin in the local direcotry is: {git_remote}\
                  \n   * Package name generated org/repo: {org_repo}\
                  \n >> Please use --source-repo-uri parameter""")
            sys.exit(1)
        else:
            org_repo = replace_ignition_gz(org_repo)
            print(' ~ Ignition found in generated org/repo assuming gz repo: '
                  + org_repo)

    # Always use github.com
    return f"https://github.com/{org_repo}.git"  # NOQA


def generate_source_params(args):
    params = {}
    # 1. NIGHTLY (launch builders):
    #     Pass the nightly branch if NIGHTLY enabled
    # 2. Launch builders
    #     Using args.source_tarball_uri if present
    # 3. Launch source jobs (SOURCE_REPO_URI)
    #     1.1 using args.source_repo_uri (if it was passed)
    #     1.2 autogenerating it

    if NIGHTLY:
        params['SOURCE_TARBALL_URI'] = args.nightly_branch
    elif args.source_tarball_uri:
        params['SOURCE_TARBALL_URI'] = args.source_tarball_uri
        if args.source_tarball_sha256:
            params['SOURCE_TARBALL_SHA256'] = args.source_tarball_sha256
    else:
        params['SOURCE_REPO_URI'] = \
            args.source_repo_uri if args.source_repo_uri else \
            generate_source_repository_uri(args)

    return params


def build_credentials_header(auth_input_arg: str = ""):
    if auth_input_arg:
        if len(auth_input_arg.split(':')) != 2:
            error("Auth string is not in the form of 'user:token' ")
        username, api_token = auth_input_arg.split(':')
    else:
        username, api_token = get_credentials(JENKINS_URL)
        if not username:
            error("No username found in credentials file")

    return make_headers(basic_auth=f'{username}:{api_token}')


def check_credentials(auth_input_arg: str = ""):
    http = urllib3.PoolManager()
    response = http.request('GET',
                            JENKINS_URL,
                            headers=build_credentials_header(auth_input_arg))
    if response.status != 200:
        print(f"Crendentials error: {response.status}: {response.reason}")
        http.clear()
        exit(1)


def call_jenkins_build(job_name: str,
                       params: dict,
                       output_string: str,
                       search_description_help: str,
                       auth_input_arg: str):
    # Only to help user feedback this block
    help_url = f'{JENKINS_URL}/job/{job_name}'
    if search_description_help:
        search_param = urllib.parse.urlencode(
                {'search': search_description_help})
        help_url += f'?{search_param}'
    print(f" + Releasing {output_string} in {help_url}")
    # Real action happen here
    params_query = urllib.parse.urlencode(params)
    url = '%s/job/%s/buildWithParameters?%s' % (JENKINS_URL,
                                                job_name,
                                                params_query)
    print_only_dbg(f" -- {output_string}: {url}")

    if not DRY_RUN:
        http = urllib3.PoolManager()
        try:
            response = \
                http.request('POST',
                             url,
                             headers=build_credentials_header(auth_input_arg))
            # 201 code is "created", it is the expected return of POST
            if response.status != 201:
                http.clear()
                error(f"{response.status}: {response.reason}")
        except RequestError as e:
            http.clear()
            error(f"An error occurred in the http request: {e}")
        except Exception as e:
            http.clear()
            error(f"An unexpected error occurred: {e}")


def display_help_job_chain_for_source_calls(args):
    # Encode the different ways using in the job descriptions to filter builds
    #   - "package version" in repository_uploader_packages
    #   - "packages/version-rev" in _releasepy
    url_search_params = urllib.parse.urlencode(
        {'search':
            f'{args.package_alias} {args.version}'})
    pkgs_upload_check_url = \
        f'{JENKINS_URL}/job/repository_uploader_packages/?{url_search_params}'
    rel_search_params = urllib.parse.urlencode(
        {'search':
            f'{args.package}/{args.version}-{args.release_version}'})
    releasepy_check_url = \
        f'{JENKINS_URL}/job/_releasepy/?{rel_search_params}'
    print('\tINFO: After the source job finished, the release process will trigger:\n'
          '\t  * Source upload:'
          f'{pkgs_upload_check_url}\n'
          '\t  * Builders using release.py --source-tarball-uri:'
          f'{releasepy_check_url}')


def get_collections_for_package(package_name, version, extra_args=()) -> list:
    script_directory = os.path.dirname(os.path.abspath(sys.argv[0]))
    helper_script = f'{script_directory}/jenkins-scripts/dsl/tools/get_collections_from_package_and_version.py'
    collection_yaml = f'{script_directory}/jenkins-scripts/dsl/gz-collections.yaml'
    cmd = [helper_script, *extra_args,
           get_canonical_package_name(package_name),
           version,
           collection_yaml]
    try:
        _out, _err = check_call(cmd, IGNORE_DRY_RUN)
    except ErrorNoOutput:
        # no output is a valid result
        _out = b""
        _err = ""
    else:
        if _err:
            print(f"An error happened running get_collections_from_package_and_version: {_err}")
            sys.exit(1)

    collection_list = _out.decode().strip().split(' ')
    return collection_list


def get_expected_branches(args) -> list:
    # gz-collections.yaml uses gz names: ign-gazebo6 -> gz-sim6
    branches = get_collections_for_package(
        replace_ignition_gz(args.package_alias), args.version, ['--branches'])
    return [b for b in branches if b]


def get_vendor_github_repo(package_name) -> str:
    canonical_name = get_canonical_package_name(package_name)
    return f"gazebo-release/{canonical_name.replace('-', '_')}_vendor"


def get_vendor_repo_url(package_name) -> str:
    # Clone needs ssh for real pushing operations. In simulation prefer https to avoid
    # unexpected pushes and facilitate testing
    protocol = 'https://github.com' if DRY_RUN else 'ssh://git@github.com'
    return f"{protocol}/{get_vendor_github_repo(package_name)}"


def prepare_vendor_pr_temp_workspace(package_name, ros_distro, ws_dir) -> Tuple[str, str, str]:
    gz_vendor_tool = os.path.join(ws_dir, "gz_vendor")
    # Create virtualenv for vendor dependencies
    venv_dir = os.path.join(ws_dir, "venv")
    venv.create(venv_dir, system_site_packages=True, with_pip=True)
    subprocess.run([os.path.join(venv_dir, 'bin', 'pip3'), 'install', '-q',
                    'jinja2==3.1.2',
                    'catkin_pkg==1.0.0'])
    cmd = ['git', 'clone', '-q',
           'https://github.com/gazebo-tooling/gz_vendor/',
           gz_vendor_tool]
    _, _err_tool = check_call(cmd, IGNORE_DRY_RUN)
    gz_vendor_repo = os.path.join(ws_dir, 'gz_vendor_repo')
    cmd = ['git', 'clone', '-q', '-b', ros_distro,
           get_vendor_repo_url(package_name),
           gz_vendor_repo]
    _, _err_repo = check_call(cmd, IGNORE_DRY_RUN)
    if _err_tool or _err_repo:
        print("Problems with cloning vendor and tool repos:")
        print(f"{_err_tool} {_err_repo}")
        sys.exit(1)

    return gz_vendor_tool, gz_vendor_repo, venv_dir


def execute_update_vendor_package_tool(vendor_tool_path,
                                       vendor_repo_path,
                                       vendor_venv) -> None:
    # The source repository when releasing matches the
    src_repo = os.getcwd()
    try:
        src_repo = os.environ['_RELEASEPY_TEST_SOURCE_REPO']
    except KeyError:
        pass

    run_cmd = [os.path.join(vendor_venv, 'bin', 'python3'),
               f"{vendor_tool_path}/create_gz_vendor_pkg/create_vendor_package.py",
               f"{os.path.join(src_repo, 'package.xml')}",
               '--output_dir', vendor_repo_path]
    _, _err_run = check_call(run_cmd, IGNORE_DRY_RUN)
    if _err_run:
        print("Problems running the create_vendor_package.py script:")
        print(_err_run.decode())
        sys.exit(1)


def create_pr_for_vendor_package(args, repo_path, base_branch) -> str:
    cmd_diff = ['git', "-C", repo_path, 'diff']
    _out, _ = check_call(cmd_diff, IGNORE_DRY_RUN)
    if not _out.decode():
        return 'vendor tool did not produce any change, avoid the PR'

    branch_name = f'releasepy/{base_branch}/{args.version}'
    vendor_repo = get_vendor_repo_url(args.package)
    branch_cmd = ['git', "-C", repo_path,
                  'checkout', '-b',  branch_name]
    _, _ = check_call(branch_cmd, IGNORE_DRY_RUN)
    commit_cmd = ['git', "-C", repo_path,
                  'commit',
                  '-m', f'Bump version to {args.version}',
                  '--all']
    _, _ = check_call(commit_cmd)
    push_cmd = ['git', "-C", repo_path,
                'push', '--force',
                vendor_repo, branch_name]
    _, _ = check_call(push_cmd)
    pr_cmd = ['gh', 'pr', 'create',
              '--base', base_branch,
              '--head', branch_name,
              '--title', f'Bump version to {args.version}',
              '--body', 'PR automatically created by release.py']
    try:
        _out, _err = check_call(pr_cmd, cwd=repo_path)
    except ErrorAlreadyExists:
        return f'there is already a PR for the branch: {branch_name} .'\
                'Please check it out manuallly.'

    if _err:
        print("Problems creating the PR for the vendor package:")
        print(_err.decode())
        sys.exit(1)

    if DRY_RUN:
        return ' (skipped the creation on --dry-run)'

    return _out.decode()


def create_pr_in_gz_vendor_repo(args, ros_distro) -> str:
    pr_msg = ''
    with tempfile.TemporaryDirectory() as ws_dir:
        ws_dir = tempfile.mkdtemp()
        # Prepare the temporary workspace
        vendor_tool_path, vendor_repo_path, venv_dir = \
            prepare_vendor_pr_temp_workspace(args.package, ros_distro, ws_dir)
        # Run updating script on the temporary workspace
        execute_update_vendor_package_tool(
            vendor_tool_path, vendor_repo_path, venv_dir)
        # Commits and PR creation
        pr_msg = create_pr_for_vendor_package(
            args, vendor_repo_path, ros_distro)

    return pr_msg


def is_gz_metapackage(package):
    return package.replace('gz-', '') in ROS_VENDOR


def get_ros_vendor_targets(args) -> list:
    # (collection, ros_distro) pairs with a vendor package to update.
    # Only create ros vendor updates for stable releases
    if PRERELEASE or NIGHTLY or is_gz_metapackage(args.package):
        return []
    return [(collection, ros_distro)
            for collection in get_collections_for_package(args.package,
                                                          args.version)
            if collection in ROS_VENDOR
            for ros_distro in ROS_VENDOR[collection]]


def process_ros_vendor_package(args):
    if PRERELEASE or NIGHTLY:
        return
    print("ROS vendor packages that can be updated:")
    if is_gz_metapackage(args.package):
        print(" - There are no gz metapackages in ROS")
        return
    for collection, ros_distro in get_ros_vendor_targets(args):
        print(f" * Github {get_vendor_github_repo(args.package)} "
              f"part of {collection} in ROS 2 {ros_distro}")
        print("   + Preparing a PR: ", end='', flush=True)
        pr_url = create_pr_in_gz_vendor_repo(args, ros_distro)
        print(pr_url)


def get_linux_build_targets(ubuntu_distros, debian_distros) -> list:
    # (linux_distro, distro, arch) for each -debbuilder call
    targets = []
    for l in LINUX_DISTROS:
        if (l == 'ubuntu'):
            distros_dic = ubuntu_distros or {}
        elif (l == 'debian'):
            if (PRERELEASE or NIGHTLY):
                continue
            if not debian_distros:
                continue
            distros_dic = debian_distros
        else:
            error("Distro not supported in code")

        for d in distros_dic:
            for a in distros_dic[d]:
                # Filter prerelease and nightly architectures
                if (PRERELEASE or NIGHTLY):
                    if (a == 'arm64'):
                        continue
                # No sid releases for arm64 lack of docker image
                # https://hub.docker.com/r/aarch64/debian/ fails on Jenkins
                if (a == 'arm64' and d == 'sid'):
                    continue
                targets.append((l, d, a))
    return targets


def format_linux_build_targets(targets) -> list:
    lines = []
    for l in LINUX_DISTROS:
        platforms = [f'{d}/{a}' for (t_l, d, a) in targets if t_l == l]
        if platforms:
            lines.append(f"{l} {' '.join(platforms)}")
    return lines


def print_release_plan(args, params, linux_targets):
    package_alias_force_gz = replace_ignition_gz(args.package_alias)
    indent = ' ' * 14

    header = f"Release plan for {args.package} {args.version}-{args.release_version}" \
             f" ({args.upload_to_repository})"
    if tags_local_repo(args):
        # Informative only, the checkout checks validate the checkout
        head = [subprocess.run(['git', 'rev-parse', flag, 'HEAD'],
                               capture_output=True, text=True)
                for flag in ('--abbrev-ref', '--short')]
        if all(r.returncode == 0 for r in head):
            branch, commit = (r.stdout.strip() for r in head)
            header += f"  [branch {branch} @ {commit}]"
    print(f"\n{header}\n")
    if args.inferred:
        print(f"  {'inferred':<12}{', '.join(args.inferred)}")

    debbuilder = f"{package_alias_force_gz}-debbuilder"
    linux_lines = format_linux_build_targets(linux_targets) or ['(none)']
    if 'SOURCE_REPO_URI' in params:
        if tags_local_repo(args):
            print(f"  {'tag':<12}{get_tag_name(args)} (local HEAD)"
                  f" -> {params['SOURCE_REPO_URI']}")
        else:
            print(f"  {'ref':<12}{args.source_repo_ref} in {params['SOURCE_REPO_URI']}")
        print(f"  {'source job':<12}{package_alias_force_gz}-source"
              f"  UPLOAD_TO_REPO={params['UPLOAD_TO_REPO']}"
              f"  RELEASE_REPO_BRANCH={params['RELEASE_REPO_BRANCH']}")
        print(f"  {'_releasepy':<12}will call {debbuilder} for:")
        for line in linux_lines:
            print(f"{indent}{line}")
    else:
        print(f"  {'source':<12}{params['SOURCE_TARBALL_URI']}")
        print(f"  {'debbuilder':<12}{debbuilder} for:")
        for line in linux_lines:
            print(f"{indent}{line}")

    if not NIGHTLY and not args.bump_rev_linux_only:
        print(f"  {'brew':<12}PR to osrf/homebrew-simulation"
              f" ({args.package_alias} {args.version})")

    if 'SOURCE_REPO_URI' in params:
        vendor_targets = get_ros_vendor_targets(args)
        if vendor_targets:
            ros_distros = ', '.join(r for _, r in vendor_targets)
            collections = ', '.join(sorted({c for c, _ in vendor_targets}))
            print(f"  {'ros vendor':<12}{get_vendor_github_repo(args.package)}:"
                  f" PR on {ros_distros}  (collection {collections})")
    print()


def confirm_release(args):
    # Only ask when the release was inferred from the checkout, explicit
    # arguments keep the scripted (Jenkins) behavior
    if args.dry_run or args.yes or \
            not {'package', 'version'} & set(args.inferred):
        return
    if not sys.stdin.isatty():
        error("Package or version were inferred from the checkout and there"
              " is no terminal to confirm. Use --yes or pass <package> <version>")
    answer = input("Proceed? [y/N] ")
    if answer.strip().lower() not in ('y', 'yes'):
        print("Aborted: nothing was tagged or triggered")
        sys.exit(1)


def go(argv):
    args = parse_args(argv)

    if args.deprecated_jenkins_token:
        error('Build token has been removed. Please generate a user token:\n'
              '  - https://gazebosim.org/docs/latest/releases-instructions/#access-and-credentials')

    # Missing package or version are read from the source checkout
    infer_from_checkout(args)
    finalize_args(args)

    # If only the process of ROS vendor package is set, just do it
    if args.bump_ros_vendor_only:
        process_ros_vendor_package(args)
        sys.exit(0)

    package_alias_force_gz = replace_ignition_gz(args.package_alias)

    print(f"Downloading releasing info for {args.package}")
    # Sanity checks and dicover supported distributions before proceed.
    repo_dir, args.release_repo_branch = download_release_repository(args.package, args.release_repo_branch)
    # Default to the revision in the -release repo, or 1
    if NIGHTLY:
        args.release_version = args.release_version or '1'
    else:
        infer_release_version(args, repo_dir)
    # The supported distros are the ones in the top level of -release repo
    ubuntu_distros = discover_distros(repo_dir)  # top level, ubuntu
    debian_distros = discover_distros(repo_dir + '/debian/')  # debian dir top level, Debian
    linux_targets = get_linux_build_targets(ubuntu_distros, debian_distros)
    if not args.no_sanity_checks:
        sanity_checks(args, repo_dir)
        if tags_local_repo(args):
            checkout_checks(args)

    params = generate_source_params(args)
    params['PACKAGE'] = args.package
    params['VERSION'] = args.version if not NIGHTLY else 'nightly'
    params['RELEASE_REPO_BRANCH'] = args.release_repo_branch
    params['PACKAGE_ALIAS'] = args.package_alias
    params['RELEASE_VERSION'] = args.release_version
    params['UPLOAD_TO_REPO'] = args.upload_to_repository
    # Assume that we want stable + own repo in the building, except when using "none"
    params['OSRF_REPOS_TO_USE'] = "stable"
    if args.upload_to_repository != "none":
        params['OSRF_REPOS_TO_USE'] += " " + args.upload_to_repository
    if args.extra_repo:
        params['OSRF_REPOS_TO_USE'] += " " + args.extra_repo

    print_release_plan(args, params, linux_targets)
    confirm_release(args)

    if args.dry_run:
        print("Simulation of jobs to be called if not dry-run:")
    else:
        print("Triggering release jobs:")
    # a) Mode nightly or builders:
    if NIGHTLY or args.source_tarball_uri:
        # RELEASING FOR BREW
        if not NIGHTLY and not args.bump_rev_linux_only:
            call_jenkins_build(GENERIC_BREW_PULLREQUEST_JOB,
                               params,
                               'Brew',
                               f'{args.package_alias}-{args.version}',
                               args.auth_input_arg)
        # RELEASING FOR LINUX
        for l, d, a in linux_targets:
            linux_platform_params = params.copy()
            linux_platform_params['ARCH'] = a
            linux_platform_params['LINUX_DISTRO'] = l
            linux_platform_params['DISTRO'] = d

            if (a == 'arm64'):
                # Need to use JENKINS_NODE_TAG parameter for large memory nodes
                # since it runs qemu emulation
                linux_platform_params['JENKINS_NODE_TAG'] = 'linux-' + a

            # The control nightly generation is done using a single machine to
            # process all gz libraries builds sequentially to avoid race
            # conditions. Note: this assumes that nodes are being tagged
            # 'linux-nightly-${ubuntu_distro} for nightly tags.
            # https://github.com/gazebo-tooling/release-tools/issues/644
            if (NIGHTLY):
                assert a == 'amd64', f'Nightly tag assumed amd64 but arch is {a}'
                linux_platform_params['JENKINS_NODE_TAG'] = 'linux-nightly-' + d
            # TODO: last parameter of providing help for -debbuilders
            # does not currently work. Somehow the string composed by
            # "-()" do not work even in the web UI directly. Real
            # string should be:
            # f"{args.version}-{args.release_version}({l}/{d}::{a})")
            call_jenkins_build(f'{package_alias_force_gz}-debbuilder',
                               linux_platform_params,
                               f"{l} {d}/{a}",
                               f"{args.version}-{args.release_version}",
                               args.auth_input_arg)
    else:
        # b) Mode generate source
        # Choose platform to run gz-source on. It will need to install gz-cmake
        # Take the first key in the supported distros since all them should be
        # able to install the needed gz-cmake.
        if ubuntu_distros:
            params['LINUX_DISTRO'] = 'ubuntu'
            params['DISTRO'] = list(ubuntu_distros.keys())[0]
        elif debian_distros:
            params['LINUX_DISTRO'] = 'debian'
            params['DISTRO'] = list(debian_distros.keys())[0]
        else:
            error("No distributions where found in the release repo")

        # Tag should not go before any method or step that can fail and just
        # before the calls to the servers.
        if not args.source_repo_ref:
            print(' * INFO: no --source-repo-existing-ref used, tag the local'
                  ' repository as the reference for the source code of the'
                  ' release')

        params['SOURCE_REPO_REF'] = tag_repo(args) \
            if not args.source_repo_ref else args.source_repo_ref

        call_jenkins_build(f'{package_alias_force_gz}-source',
                           params,
                           'Source',
                           args.version,
                           args.auth_input_arg)
        display_help_job_chain_for_source_calls(args)
        # Process the possible update of an associated ROS vendor package
        # for stable releases only
        process_ros_vendor_package(args)


if __name__ == '__main__':
    go(sys.argv)
