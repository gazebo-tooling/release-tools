#!/usr/bin/env python3

from __future__ import print_function
import subprocess
import sys
import tempfile
import os
import urllib.parse
import urllib.request
import argparse
import shutil
import re

USAGE = 'release.py <package> <version> <jenkinstoken>'
try:
    JENKINS_URL = os.environ['JENKINS_URL']
except KeyError:
    JENKINS_URL = 'http://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-debbuilder'
GENERIC_BREW_PULLREQUEST_JOB = 'generic-release-homebrew_pull_request_updater'
UPLOAD_DEST_PATTERN = 's3://osrf-distributions/%s/releases/'
DOWNLOAD_URI_PATTERN = 'https://osrf-distributions.s3.amazonaws.com/%s/releases/'

LINUX_DISTROS = ['ubuntu', 'debian']
SUPPORTED_ARCHS = ['amd64', 'armhf', 'arm64']
RELEASEPY_NO_ARCH_PREFIX = '.releasepy_NO_ARCH_'

OSRF_REPOS_SUPPORTED = "stable prerelease nightly testing"

DRY_RUN = False
NIGHTLY = False
PRERELEASE = False

IGNORE_DRY_RUN = True

GARDEN_IGN_PACKAGES = ['ign-cmake3',
                       'ign-common5',
                       'ign-fuel-tools8',
                       'ign-sim7',
                       'ign-gui7',
                       'ign-launch6',
                       'ign-math7',
                       'ign-msgs9',
                       'ign-physics6',
                       'ign-plugin2',
                       'ign-rendering7',
                       'ign-sensors7',
                       'ign-tools2',
                       'ign-transport12',
                       'ign-utils2']


class ErrorNoPermsRepo(Exception):
    pass


class ErrorNoUsernameSupplied(Exception):
    pass


class ErrorURLNotFound404(Exception):
    pass


class ErrorNoOutput(Exception):
    pass


def error(msg):
    print("\n !! " + msg + "\n")
    sys.exit(1)


def print_success(msg):
    print(" + OK " + msg)


# Remove the last character if it is a number.
# That should leave just the package name instead of packageVersion
# I.E gazebo5 -> gazebo
def get_canonical_package_name(pkg_name):
    return pkg_name.rstrip('1234567890')


def github_repo_exists(url):
    try:
        check_call(['git', 'ls-remote', '-q', '--exit-code', url], IGNORE_DRY_RUN)
    except (ErrorURLNotFound404, ErrorNoOutput):
        return False
    except Exception as e:
        error("Unexpected problem checking for git repo: " + str(e))
    return True


def generate_package_source(srcdir, builddir):
    cmake_cmd = ["cmake"]

    # configure and make package_source
    os.mkdir(builddir)
    os.chdir(builddir)
    check_call(cmake_cmd + [srcdir])
    check_call(['make', 'package_source'])


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

    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
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
    parser.add_argument('--no-generate-source-file', dest='no_source_file', action='store_true', default=False,
                        help='Do not generate source file when building')
    parser.add_argument('--upload-to-repo', dest='upload_to_repository', default="stable",
                        help='OSRF repo to upload: stable | prerelease | nightly')
    parser.add_argument('--extra-osrf-repo', dest='extra_repo', default="",
                        help='extra OSRF repository to use in the build')
    parser.add_argument('--nightly-src-branch', dest='nightly_branch', default="main",
                        help='branch in the source code repository to build the nightly from')
    parser.add_argument('--only-bump-revision-linux', dest='bump_rev_linux_only',
                        action='store_true', default=False,
                        help='Bump only revision number. Do not upload new tarball.')

    args = parser.parse_args()

    if args.package in GARDEN_IGN_PACKAGES:
        print(f"Garden packages start with gz- prefix, changing {args.package} to {args.package.replace('ign-','gz-')}",)
        args.package = args.package.replace('ign-', 'gz-')

    args.package_alias = args.package

    DRY_RUN = args.dry_run
    if args.upload_to_repository == 'nightly':
        NIGHTLY = True
    if args.upload_to_repository == 'prerelease':
        PRERELEASE = True
    # Nightly do not generate a tar.bz2 file
    if NIGHTLY:
        args.no_source_file = True

    return args


def get_release_repository_info(package):
    github_url = "https://github.com/gazebo-release/" + package + "-release"
    if (github_repo_exists(github_url)):
        return 'git', github_url


def download_release_repository(package, release_branch):
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


def sanity_package_version(repo_dir, version, release_version):
    cmd = ["find", repo_dir, "-name", "changelog", "-exec", "head", "-n", "1", "{}", ";"]
    out, _ = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.decode().split('\n'):
        if not line:
            continue
        # return full version in brackets
        full_version = line.split(' ')[1]
        # get only version (not release) in brackets
        c_version = full_version[full_version.find("(")+1:full_version.find("-")]
        c_revision = full_version[full_version.find("-")+1:full_version.rfind("~")]

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


def sanity_check_repo_name(repo_name):
    if repo_name in OSRF_REPOS_SUPPORTED:
        return

    error("Upload repo value: " + repo_name + " is not valid. stable | prerelease | nightly")


def sanity_project_package_in_stable(version, repo_name):
    if repo_name != 'stable':
        return

    if '+' in version:
        error("Detected stable repo upload using project versioning scheme (include '+' in the version)")

    if '~pre' in version:
        error("Detected stable repo upload using project versioning scheme (include '~pre' in the version)")

    return


def check_s3cmd_configuration():
    # Need to check if s3cmd is installed
    try:
        subprocess.call(["s3cmd", "--version"])
    except OSError:
        error("s3cmd command for uploading is not available. Install it using: apt-get install s3cmd")

    # Need to check if configuration for s3 exists
    s3_config = os.path.expanduser('~') + "/.s3cfg"
    if not os.path.isfile(s3_config):
        error(s3_config + " does not exists. Please configure s3: s3cmd --configure")

    return True


def sanity_checks(args, repo_dir):
    print("Safety checks:")
    sanity_package_name_underscore(args.package, args.package_alias)
    sanity_package_name(repo_dir, args.package, args.package_alias)
    sanity_check_repo_name(args.upload_to_repository)

    if not NIGHTLY:
        check_s3cmd_configuration()
        sanity_package_version(repo_dir, args.version, str(args.release_version))
        sanity_check_sdformat_versions(args.package, args.version)
        sanity_project_package_in_stable(args.version, args.upload_to_repository)

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

    print('Releasing for distributions: ')
    for k in distro_arch_list:
        print("- " + k + " (" + ', '.join(distro_arch_list[k]) + ")")

    return distro_arch_list


def check_call(cmd, ignore_dry_run=False):
    if DRY_RUN and not ignore_dry_run:
        print('Dry-run running:\n  %s\n' % (' '.join(cmd)))
        return b'', b''
    else:
        print('Running:\n  %s' % (' '.join(cmd)))
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = po.communicate()
        if po.returncode != 0:
            # bitbucket for the first one, github for the second
            if (b"404" in err) or (b"Repository not found" in err):
                raise ErrorURLNotFound404()
            if b"Permission denied" in out:
                raise ErrorNoPermsRepo()
            if b"abort: no username supplied" in err:
                raise ErrorNoUsernameSupplied()
            if not out and not err:
                # assume that call is only for getting return code
                raise ErrorNoOutput()

            # Unkown exception
            print('Error running command (%s).' % (' '.join(cmd)))
            print('stdout: %s' % (out.decode()))
            print('stderr: %s' % (err.decode()))
            raise Exception('subprocess call failed')
        return out, err


# Returns tarball name: package name/alias without versions
def create_tarball_name(args):
    return re.sub(r'[0-9]+$', '', args.package)


# Returns: sha, tarball file name, tarball full path
def create_tarball_path(tarball_name, version, builddir, dry_run):
    tarball_fname = '%s-%s.tar.bz2' % (tarball_name, version)
    # Try using the tarball_name as it is
    tarball_path = os.path.join(builddir, tarball_fname)

    if not os.path.isfile(tarball_path):
        # Try looking for special project names using underscores
        alt_tarball_name = "_".join(tarball_name.rsplit("-", 1))
        alt_tarball_fname = '%s-%s.tar.bz2' % (alt_tarball_name, version)
        alt_tarball_path = os.path.join(builddir, alt_tarball_fname)
        if (not dry_run):
            if not os.path.isfile(alt_tarball_path):
                error("Can not find a tarball at: " + tarball_path + " or at " + alt_tarball_path)
            else:
                tarball_fname = alt_tarball_fname
        tarball_path = alt_tarball_path

    out, err = check_call(['shasum', '--algorithm', '256', tarball_path])
    if err:
        error("shasum returned an error: " + err.decode())
    if isinstance(out, bytes):
        out = out.decode()

    return out.split(' ')[0], tarball_fname, tarball_path


def tag_repo(args):
    try:
        # tilde is not a valid character in git
        tag = '%s_%s' % (args.package_alias, args.version.replace('~', '-'))
        check_call(['git', 'tag', '-f', tag])
        check_call(['git', 'push', '--tags'])
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


def generate_upload_tarball(args):
    ###################################################
    # Platform-agnostic stuff.
    # The goal is to tag the repo and prepare a tarball.

    sourcedir = os.getcwd()
    tmpdir    = tempfile.mkdtemp()
    builddir  = os.path.join(tmpdir, 'build')

    # Note for bump_rev_linux_only: there are some adjustment to the tarball name
    # that are done after generating it, even if the tarball upload is not
    # needed, it should be generated to get changes in the name
    if args.bump_rev_linux_only:
        print('\nINFO: bump revision is enabled. It needs to generate a local tarball although it will not upload it')

    # Check for uncommitted changes; abort if present
    cmd = ['git', 'diff-index', 'HEAD']
    out, err = check_call(cmd)
    if out:
        print('git says that you have uncommitted changes')
        print('Please clean up your working copy so that "%s" outputs nothing' % (' '.join(cmd)))
        print('stdout: %s' % (out.decode()))
        sys.exit(1)

    # Make a clean copy, to avoid pulling in other stuff that the user has
    # sitting in the working copy.
    srcdir = os.path.join(tmpdir, 'src')
    os.mkdir(srcdir)
    tmp_tar = os.path.join(tmpdir, 'orig.tar')
    check_call(['git', 'archive', '--format=tar', 'HEAD', '-o', tmp_tar])
    check_call(['tar', 'xf', tmp_tar, '-C', srcdir])
    if not args.dry_run:
        os.remove(tmp_tar)

    # use cmake to generate package_source
    generate_package_source(srcdir, builddir)
    # For ignition, we use the alias without version numbers as package name
    tarball_name = re.sub(r'[0-9]+$', '', args.package_alias)
    tarball_sha, tarball_fname, tarball_path = create_tarball_path(tarball_name, args.version, builddir, args.dry_run)

    # If we're releasing under a different name, then rename the tarball (the
    # package itself doesn't know anything about this).
    if args.package != args.package_alias:
        tarball_fname = '%s-%s.tar.bz2' % (args.package_alias, args.version)
        if (not args.dry_run):
            dest_file = os.path.join(builddir, tarball_fname)
            # Do not copy if files are the same
            if not (tarball_path == dest_file):
                shutil.copyfile(tarball_path, dest_file)
                tarball_path = dest_file

    s3_tarball_directory = UPLOAD_DEST_PATTERN % get_canonical_package_name(args.package)
    source_tarball_uri = DOWNLOAD_URI_PATTERN % get_canonical_package_name(args.package) + tarball_fname

    # If the release only bump revision does not need to upload tarball but
    # checkout that the one that should be already uploaded exists
    if args.bump_rev_linux_only:
        if urllib.request.urlopen(source_tarball_uri).getcode() == '404':
            print('Can not find tarball: %s' % (source_tarball_uri))
            sys.exit(1)
    else:
        check_call(['s3cmd', 'sync', tarball_path, s3_tarball_directory])
        shutil.rmtree(tmpdir)

        # Tag repo
        os.chdir(sourcedir)
        _ = tag_repo(args)

    # TODO: Consider auto-updating the Ubuntu changelog.  It requires
    # cloning the <package>-release repo, making a change, and pushing it back.
    # Until we do that, the user must have first updated it manually.
    return source_tarball_uri, tarball_sha


def call_jenkins_build(job_name, params, output_string):
    params_query = urllib.parse.urlencode(params)
    url = '%s/job/%s/buildWithParameters?%s' % (JENKINS_URL,
                                                job_name,
                                                params_query)
    print(f"- {output_string}: {url}")
    if not DRY_RUN:
        urllib.request.urlopen(url)


def go(argv):
    args = parse_args(argv)

    # Default to release 1 if not present
    if not args.release_version:
        args.release_version = 1

    # Sanity checks and dicover supported distributions before proceed.
    repo_dir, args.release_repo_branch = download_release_repository(args.package, args.release_repo_branch)
    # The supported distros are the ones in the top level of -release repo
    ubuntu_distros = discover_distros(repo_dir)  # top level, ubuntu
    debian_distros = discover_distros(repo_dir + '/debian/')  # debian dir top level, Debian
    if not args.no_sanity_checks:
        sanity_checks(args, repo_dir)

    source_tarball_uri = ''
    source_tarball_sha = ''

    # Do not generate source file if not needed or impossible
    if not args.no_source_file:
        source_tarball_uri, source_tarball_sha = generate_upload_tarball(args)

    # Kick off Jenkins jobs
    params = {}
    params['token'] = args.jenkins_token
    params['PACKAGE'] = args.package
    params['VERSION'] = args.version
    params['SOURCE_TARBALL_URI'] = source_tarball_uri
    params['SOURCE_TARBALL_SHA'] = source_tarball_sha
    params['RELEASE_REPO_BRANCH'] = args.release_repo_branch
    params['PACKAGE_ALIAS'] = args.package_alias
    params['RELEASE_VERSION'] = args.release_version
    params['UPLOAD_TO_REPO'] = args.upload_to_repository
    # Assume that we want stable + own repo in the building
    params['OSRF_REPOS_TO_USE'] = "stable " + args.upload_to_repository
    if args.extra_repo:
        params['OSRF_REPOS_TO_USE'] += " " + args.extra_repo

    if NIGHTLY:
        params['VERSION'] = 'nightly'
        # reuse SOURCE_TARBALL_URI to indicate the nightly branch
        # name must be modified in the future
        params['SOURCE_TARBALL_URI'] = args.nightly_branch

    job_name = JOB_NAME_PATTERN % (args.package)

    # RELEASING FOR BREW
    if not NIGHTLY and not args.bump_rev_linux_only:
        call_jenkins_build(GENERIC_BREW_PULLREQUEST_JOB,
                           params, 'Brew')

    # RELEASING FOR LINUX
    for l in LINUX_DISTROS:
        if (l == 'ubuntu'):
            distros_dic = ubuntu_distros
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
                    if (a == 'armhf' or a == 'arm64'):
                        continue

                linux_platform_params = params.copy()
                linux_platform_params['ARCH'] = a
                linux_platform_params['LINUX_DISTRO'] = l
                linux_platform_params['DISTRO'] = d

                if (a == 'armhf' or a == 'arm64'):
                    # No sid releases for arm64/armhf lack of docker image
                    # https://hub.docker.com/r/aarch64/debian/ fails on Jenkins
                    if (d == 'sid'):
                        continue
                    # Need to use JENKINS_NODE_TAG parameter for large memory nodes
                    # since it runs qemu emulation
                    linux_platform_params['JENKINS_NODE_TAG'] = 'linux-' + a
                elif ('ignition-physics' in args.package_alias) or \
                     ('gz-physics' in args.package_alias):
                    linux_platform_params['JENKINS_NODE_TAG'] = 'large-memory'

                # control nightly generation using a single machine to process
                # all distribution builds to avoid race conditions. Note: this
                # assumes that large-memory nodes are beind used for nightly
                # tags.
                # https://github.com/gazebo-tooling/release-tools/issues/644
                if (NIGHTLY):
                    assert a == 'amd64', f'Nightly tag assumed amd64 but arch is {a}'
                    linux_platform_params['JENKINS_NODE_TAG'] = 'linux-nightly-' + d

                call_jenkins_build(job_name, linux_platform_params, 'Linux')


if __name__ == '__main__':
    go(sys.argv)
