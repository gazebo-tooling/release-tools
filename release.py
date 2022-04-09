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
JOB_NAME_UPSTREAM_PATTERN = 'upstream-%s-debbuilder'
GENERIC_BREW_PULLREQUEST_JOB='generic-release-homebrew_pull_request_updater'
UPLOAD_DEST_PATTERN = 's3://osrf-distributions/%s/releases/'
DOWNLOAD_URI_PATTERN = 'https://osrf-distributions.s3.amazonaws.com/%s/releases/'

LINUX_DISTROS = [ 'ubuntu', 'debian' ]
SUPPORTED_ARCHS = ['amd64', 'i386', 'armhf', 'arm64']
RELEASEPY_NO_ARCH_PREFIX = '.releasepy_NO_ARCH_'

# Ubuntu distributions are automatically taken from the top directory of
# the release repositories, when needed.
UBUNTU_DISTROS = []

OSRF_REPOS_SUPPORTED = "stable prerelease nightly"
OSRF_REPOS_SELF_CONTAINED = ""

DRY_RUN = False
NIGHTLY = False
PRERELEASE = False
UPSTREAM = False
NO_SRC_FILE = False

IGNORE_DRY_RUN = True

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
    print("     + OK " + msg)

# Remove the last character if it is a number.
# That should leave just the package name instead of packageVersion
# I.E gazebo5 -> gazebo
def get_canonical_package_name(pkg_name):
     return pkg_name.rstrip('1234567890')

def is_catkin_package():
    return os.path.isfile("package.xml")

def github_repo_exists(url):
    try:
        check_call(['git', 'ls-remote', '-q', '--exit-code', url], IGNORE_DRY_RUN)
    except (ErrorURLNotFound404, ErrorNoOutput) as e:
        return False
    except Exception as e:
        error("Unexpected problem checking for git repo: " + str(e))
    return True

def generate_package_source(srcdir, builddir):
    cmake_cmd = ["cmake"]

    if is_catkin_package():
        cmake_cmd = cmake_cmd + ['-DCATKIN_BUILD_BINARY_PACKAGE="1"']

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
    except Exception as e:
        return False

def parse_args(argv):
    global DRY_RUN
    global NIGHTLY
    global PRERELEASE
    global UPSTREAM
    global NO_SRC_FILE

    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    parser.add_argument('-u', '--upstream', dest='upstream', action='store_true', default=False,
                        help='release non OSRF software (do not generate and upload source tar.bz)')
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
    parser.add_argument('--no-ignition-auto', dest='no_ignition_auto', action='store_true', default=False,
                        help='Use package name to create --package-alias if package name starts with ign-')
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

    args.package_alias = args.package
    # If ignition auto is enabled, replace ign- with ignition- at the beginning
    if not args.no_ignition_auto and args.package.startswith('ign-'):
        args.package_alias = args.package.replace('ign-', 'ignition-')

    DRY_RUN = args.dry_run
    UPSTREAM = args.upstream
    NO_SRC_FILE = args.no_source_file
    UPLOAD_REPO = args.upload_to_repository
    if args.upload_to_repository == 'nightly':
        NIGHTLY = True
        NIGHTLY_BRANCH = args.nightly_branch
    if args.upload_to_repository == 'prerelease':
        PRERELEASE = True
    # Upstream and nightly do not generate a tar.bz2 file
    if args.upstream or NIGHTLY:
        NO_SRC_FILE = True
        args.no_source_file = True

    return args

def get_release_repository_info(package):
    # Do not use git@github method since it fails in non existant repositories
    # asking for stdin user/pass. Same happen if no user/pass is provided
    # using the fake foo:foo here seems to work
    github_test_url = "https://foo:foo@github.com/ignition-release/" + package + "-release"
    if (github_repo_exists(github_test_url)):
        github_url = "https://github.com/ignition-release/" + package + "-release"
        return 'git', github_url

    error("release repository not found in github.com/ignition-release")

def download_release_repository(package, release_branch):
    vcs, url = get_release_repository_info(package)
    release_tmp_dir = tempfile.mkdtemp()

    if vcs == "git" and release_branch == "default":
        release_branch = "master"

    # If main branch exists, prefer it over master
    if release_branch == "master":
        if exists_main_branch(url):
            print_success('Found main branch in repo, use it instead master')
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

    cmd = ["find", repo_dir, "-name", "changelog","-exec","head","-n","1","{}",";"]
    out, err = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.decode().split('\n'):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[0] != expected_name:
            error("Error in changelog package name or alias: " + line)

    cmd = ["find", repo_dir, "-name", "control","-exec","grep","-H","Source:","{}",";"]
    out, err = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.decode().split('\n'):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[2] != expected_name:
            error("Error in source package. File:  " + line.partition(' ')[1] + ". Got " + line.partition(' ')[2] + " expected " + expected_name)

    print_success("Package names in changelog and control")

def sanity_package_version(repo_dir, version, release_version):
    cmd = ["find", repo_dir, "-name", "changelog","-exec","head","-n","1","{}",";"]
    out, err = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.decode().split('\n'):
        if not line:
            continue
        # return full version in brackets
        full_version=line.split(' ')[1]
        # get only version (not release) in brackets
        c_version=full_version[full_version.find("(")+1:full_version.find("-")]
        c_revision=full_version[full_version.find("-")+1:full_version.rfind("~")]

        if c_version != version:
            error("Error in package version. Repo version: " + c_version + " Provided version: " + version)

        if c_revision != release_version:
            error("Error in package release version. Expected " + release_version + " in line " + full_version)

    print_success("Package versions in changelog")
    print_success("Package release versions in changelog")

def sanity_check_gazebo_versions(package, version):
    if package == 'gazebo':
        if int(version[0]) > 1:
            error("Error in gazebo version. Please use 'gazebo-current' package for gazebo 2")
    elif package == 'gazebo-current':
        if int(version[0]) < 2:
            error("Error in gazebo-current version. Please use 'gazebo' package for gazebo 1.x")
    else:
        return

    print_success("Gazebo version in proper gazebo package")

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

def sanity_use_prerelease_branch(release_branch):
    if release_branch == 'prerelease':
        error("The use of prerelease branch is now deprecated. Please check internal wiki instructions")

    return

def check_s3cmd_configuration():
    # Need to check if s3cmd is installed
    try:
        subprocess.call(["s3cmd", "--version"])
    except OSError as e:
        error("s3cmd command for uploading is not available. Install it using: apt-get install s3cmd")

    # Need to check if configuration for s3 exists
    s3_config = os.path.expanduser('~') + "/.s3cfg"
    if not os.path.isfile(s3_config):
        error(s3_config + " does not exists. Please configure s3: s3cmd --configure")

    return True

def sanity_checks(args, repo_dir):
    sanity_package_name_underscore(args.package, args.package_alias)
    sanity_package_name(repo_dir, args.package, args.package_alias)
    sanity_check_repo_name(args.upload_to_repository)
    sanity_use_prerelease_branch(args.release_repo_branch)

    if not NIGHTLY:
        check_s3cmd_configuration()
        sanity_package_version(repo_dir, args.version, str(args.release_version))
        sanity_check_gazebo_versions(args.package, args.version)
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

    if '.git' in subdirs: subdirs.remove('.git')
    # remove ubuntu (common stuff) and debian (new supported distro at top level)
    if 'ubuntu' in subdirs: subdirs.remove('ubuntu')
    if 'debian' in subdirs: subdirs.remove('debian')
    # Some releasing methods use patches/ in root
    if 'patches' in subdirs: subdirs.remove('patches')

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
        print( "- " + k + " (" + ', '.join(distro_arch_list[k]) +")")

    return distro_arch_list

def check_call(cmd, ignore_dry_run = False):
    if ignore_dry_run:
        # Commands that do not change anything in repo level
        print('Dry-run running:\n  %s\n'%(' '.join(cmd)))
    else:
        print('Running:\n  %s'%(' '.join(cmd)))
    if DRY_RUN and not ignore_dry_run:
        return '', ''
    else:
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
            print('Error running command (%s).'%(' '.join(cmd)))
            print('stdout: %s'%(out.decode()))
            print('stderr: %s'%(err.decode()))
            raise Exception('subprocess call failed')
        return out, err


# Returns tarball name: package name/alias without versions
def create_tarball_name(args):
    # For ignition, we use the package_alias instead of package
    return re.sub(r'[0-9]+$', '',
                  args.package if not IGN_REPO else args.package_alias)


# Returns: sha, tarball file name, tarball full path
def create_tarball_path(tarball_name, version, builddir, dry_run):
    tarball_fname = '%s-%s.tar.bz2'%(tarball_name, version)
    # Try using the tarball_name as it is
    tarball_path = os.path.join(builddir, tarball_fname)

    if not os.path.isfile(tarball_path):
        # Try looking for special project names using underscores
        alt_tarball_name = "_".join(tarball_name.rsplit("-",1))
        alt_tarball_fname = '%s-%s.tar.bz2'%(alt_tarball_name, version)
        alt_tarball_path = os.path.join(builddir, alt_tarball_fname)
        if (not dry_run):
            if not os.path.isfile(alt_tarball_path):
                error("Can not find a tarball at: " + tarball_path + " or at " + alt_tarball_path)
        tarball_path = alt_tarball_path

    out, err = check_call(['shasum', '--algorithm', '256', tarball_path])
    if err:
        error("shasum returned an error: " + err.decode())
    if isinstance(out, bytes):
        out = out.decode()

    return out.split(' ')[0], tarball_fname, tarball_path

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
        tarball_fname = '%s-%s.tar.bz2'%(args.package_alias, args.version)
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

        try:
            # tilde is not a valid character in git
            tag = '%s_%s' % (args.package_alias, args.version.replace('~','-'))
            check_call(['git', 'tag', '-f', tag])
            check_call(['git', 'push', '--tags'])
        except ErrorNoPermsRepo as e:
            print('The Git server reports problems with permissions')
            print('The branch could be blocked by configuration if you do not have')
            print('rights to push code in default branch.')
            sys.exit(1)
        except ErrorNoUsernameSupplied as e:
            print('git tag could not be committed because you have not configured')
            print('your username. Use "git config --username" to set your username.')
            sys.exit(1)
        except Exception as e:
            print('There was a problem with pushing tags to the git repository')
            print('Do you have write perms in the repository?')
            sys.exit(1)

    # TODO: Consider auto-updating the Ubuntu changelog.  It requires
    # cloning the <package>-release repo, making a change, and pushing it back.
    # Until we do that, the user must have first updated it manually.
    return source_tarball_uri, tarball_sha


def go(argv):
    args = parse_args(argv)

    # Default to release 1 if not present
    if not args.release_version:
        args.release_version = 1

    # Sanity checks and dicover supported distributions before proceed.
    # UPSTREAM repository is not known in release-tools script
    if not UPSTREAM:
        repo_dir, args.release_repo_branch = download_release_repository(args.package, args.release_repo_branch)
        # The supported distros are the ones in the top level of -release repo
        ubuntu_distros = discover_distros(repo_dir) # top level, ubuntu
        debian_distros = discover_distros(repo_dir + '/debian/') # debian dir top level, Debian
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

    if args.upload_to_repository in OSRF_REPOS_SELF_CONTAINED:
        params['OSRF_REPOS_TO_USE'] = args.upload_to_repository

    if NIGHTLY:
        params['VERSION'] = 'nightly'
        # reuse SOURCE_TARBALL_URI to indicate the nightly branch
        # name must be modified in the future
        params['SOURCE_TARBALL_URI'] = args.nightly_branch

    if UPSTREAM:
        job_name = JOB_NAME_UPSTREAM_PATTERN%(args.package)
    else:
        job_name = JOB_NAME_PATTERN%(args.package)

    params_query = urllib.parse.urlencode(params)

    # RELEASING FOR BREW
    brew_url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL,
                                                   GENERIC_BREW_PULLREQUEST_JOB,
                                                   params_query)
    if not NIGHTLY and not args.bump_rev_linux_only:
        print('- Brew: %s' % (brew_url))
        if not DRY_RUN:
            urllib.request.urlopen(brew_url)

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
                # Only i386 for Ubuntu in Bionic
                if (a == 'i386' and l != 'debian' and d != 'bionic'):
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
                elif ('ignition-physics' in args.package_alias):
                    linux_platform_params['JENKINS_NODE_TAG'] = 'large-memory'

                if (NIGHTLY and a == 'i386'):
                    continue

                # control nightly generation using a single machine to process
                # all distribution builds to avoid race conditions. Note: this
                # assumes that large-memory nodes are beind used for nightly
                # tags.
                # https://github.com/ignition-tooling/release-tools/issues/644
                if (NIGHTLY):
                    assert a == 'amd64', f'Nightly tag assumed amd64 but arch is {a}'
                    linux_platform_params['JENKINS_NODE_TAG'] = 'linux-nightly-' + d

                linux_platform_params_query = urllib.parse.urlencode(linux_platform_params)

                url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL, job_name, linux_platform_params_query)
                print('- Linux: %s'%(url))

                if not DRY_RUN:
                    urllib.request.urlopen(url)

if __name__ == '__main__':
    go(sys.argv)
