#!/usr/bin/env python

from __future__ import print_function
import subprocess
import sys
import tempfile
import os
import urllib
import argparse
import shutil
import re

USAGE = 'release.py <package> <version> <jenkinstoken>'
JENKINS_URL = 'http://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-debbuilder'
JOB_NAME_UPSTREAM_PATTERN = 'upstream-%s-debbuilder'
GENERIC_BREW_PULLREQUEST_JOB='generic-release-homebrew_pull_request_updater'
UPLOAD_DEST_PATTERN = 's3://osrf-distributions/%s/releases/'
DOWNLOAD_URI_PATTERN = 'http://gazebosim.org/distributions/%s/releases/'

LINUX_DISTROS = [ 'ubuntu', 'debian' ]
SUPPORTED_ARCHS = ['amd64', 'i386', 'armhf', 'arm64']

# Ubuntu distributions are automatically taken from the top directory of
# the release repositories, when needed.
UBUNTU_DISTROS = []

OSRF_REPOS_SUPPORTED="stable prerelease nightly mentor2 haptix-pre"
OSRF_REPOS_SELF_CONTAINED="mentor2"

DRY_RUN = False
NIGHTLY = False
PRERELEASE = False
UPSTREAM = False
NO_SRC_FILE = False
DRCSIM_MULTIROS = False
IGN_REPO = False

IGNORE_DRY_RUN = True

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

def generate_package_source(srcdir, builddir):
    cmake_cmd = ["cmake"]

    if is_catkin_package():
        cmake_cmd = cmake_cmd + ['-DCATKIN_BUILD_BINARY_PACKAGE="1"']

    # configure and make package_source
    os.mkdir(builddir)
    os.chdir(builddir)
    check_call(cmake_cmd + [srcdir])
    check_call(['make', 'package_source'])

def parse_args(argv):
    global DRY_RUN
    global NIGHTLY
    global PRERELEASE
    global UPSTREAM
    global NO_SRC_FILE
    global DRCSIM_MULTIROS
    global IGN_REPO

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
                        default='default',
                        help='which version of the accompanying -release repo to use (if not default)')
    parser.add_argument('-r', '--release-version', dest='release_version',
                        default=None,
                        help='Release version suffix; usually 1 (e.g., 1')
    parser.add_argument('--drcsim-multiros', dest='drcsim_multiros', action='store_true', default=False,
                        help='To be used with drcsim, osrf-common and sandia-hand. Generate ROS_DISTRO based on drcsim support (i.e: ')
    parser.add_argument('--no-sanity-checks', dest='no_sanity_checks', action='store_true', default=False,
                        help='no-sanity-checks; i.e. skip sanity checks commands')
    parser.add_argument('--no-generate-source-file', dest='no_source_file', action='store_true', default=False,
                        help='Do not generate source file when building')
    parser.add_argument('--ignition-repo', dest='ignition_repo', action='store_true', default=False,
                        help='use ignition robotics URL repositories instead of OSRF')
    parser.add_argument('--upload-to-repo', dest='upload_to_repository', default="stable",
                        help='OSRF repo to upload: stable | prerelease | nightly')


    args = parser.parse_args()
    if not args.package_alias:
        args.package_alias = args.package
    DRY_RUN = args.dry_run
    UPSTREAM = args.upstream
    NO_SRC_FILE = args.no_source_file
    DRCSIM_MULTIROS = args.drcsim_multiros
    IGN_REPO = args.ignition_repo
    UPLOAD_REPO = args.upload_to_repository
    if args.upload_to_repository == 'nightly':
        NIGHTLY = True
    if args.upload_to_repository == 'prerelease':
        PRERELEASE = True
    # Upstream and nightly do not generate a tar.bz2 file
    if args.upstream or NIGHTLY:
        NO_SRC_FILE = True
        args.no_source_file = True

    return args

def get_release_repository_URL(package):
    repo = "osrf"
    if IGN_REPO:
        repo = "ignitionrobotics"

    return "https://bitbucket.org/" + repo + "/" + package + "-release"

def download_release_repository(package, release_branch):
    url = get_release_repository_URL(package)
    release_tmp_dir = tempfile.mkdtemp()
    cmd = [ "hg", "clone", "-b", release_branch, url, release_tmp_dir]
    check_call(cmd, IGNORE_DRY_RUN)
    return release_tmp_dir

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
    for line in out.split("\n"):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[0] != expected_name:
            error("Error in changelog package name or alias: " + line)

    cmd = ["find", repo_dir, "-name", "control","-exec","grep","-H","Source:","{}",";"]
    out, err = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.split("\n"):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[2] != expected_name:
            error("Error in source package. File:  " + line.partition(' ')[1] + ". Got " + line.partition(' ')[2] + " expected " + expected_name)

    print_success("Package names in changelog and control")

def sanity_package_version(repo_dir, version, release_version):
    cmd = ["find", repo_dir, "-name", "changelog","-exec","head","-n","1","{}",";"]
    out, err = check_call(cmd, IGNORE_DRY_RUN)
    for line in out.split("\n"):
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

    if not '+' in version:
        return


    error("Detected stable repo upload using project versioning scheme (include '+' in the version)")

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
    check_s3cmd_configuration()

    sanity_package_name_underscore(args.package, args.package_alias)
    sanity_package_name(repo_dir, args.package, args.package_alias)
    sanity_check_repo_name(args.upload_to_repository)
    sanity_use_prerelease_branch(args.release_repo_branch)

    if not NIGHTLY:
        sanity_package_version(repo_dir, args.version, str(args.release_version))
        sanity_check_gazebo_versions(args.package, args.version)
        sanity_check_sdformat_versions(args.package, args.version)
        sanity_project_package_in_stable(args.version, args.upload_to_repository)

    shutil.rmtree(repo_dir)

def discover_ubuntu_distros(args, repo_dir):
    subdirs =  os.walk(repo_dir).next()[1]
    subdirs.remove('.hg')
    # remove ubuntu (common stuff) and debian (new supported distro at top level)
    if 'ubuntu' in subdirs: subdirs.remove('ubuntu')
    if 'debian' in subdirs: subdirs.remove('debian')
    # Some releasing methods use patches/ in root
    if 'patches' in subdirs: subdirs.remove('patches')

    if not subdirs:
        error('Can not find distributions directories in the -release repo')

    print('Releasing for distributions: ' + ', '.join(subdirs))

    return subdirs

def discover_debian_distros(args, repo_dir):
    subdirs =  os.walk(repo_dir+ '/debian/').next()[1]

    if not subdirs:
        error('Can not find debian distributions directories in the -release repo')

    print('Releasing for distributions: ' + ', '.join(subdirs))

    return subdirs


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
            print('Error running command (%s).'%(' '.join(cmd)))
            print('stdout: %s'%(out))
            print('stderr: %s'%(err))
            raise Exception('subprocess call failed')
        return out, err

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

    shasum_out_err = check_call(['shasum', '--algorithm', '256', tarball_path])
    return shasum_out_err[0].split(' ')[0], tarball_fname, tarball_path

def generate_upload_tarball(args):
    ###################################################
    # Platform-agnostic stuff.
    # The goal is to tag the repo and prepare a tarball.

    sourcedir = os.getcwd()
    tmpdir    = tempfile.mkdtemp()
    builddir  = os.path.join(tmpdir, 'build')
    # Put the hg-specific stuff in a try block, to allow for a git repo
    try:
        # Check for uncommitted changes; abort if present
        cmd = ['hg', 'status', '-q']
        out, err = check_call(cmd)
        if len(out) != 0:
            print('Mercurial says that you have uncommitted changes.  Please clean up your working copy so that "%s" outputs nothing'%(' '.join(cmd)))
            print('stdout: %s'%(out))
            sys.exit(1)

        # Make a clean copy, to avoid pulling in other stuff that the user has
        # sitting in the working copy
        srcdir = os.path.join(tmpdir, 'src')
        check_call(['hg', 'archive', srcdir])
    except Exception as e:
        # Assume that it's git and that we'll just use the CWD
        srcdir = os.getcwd()

    # use cmake to generate package_source
    generate_package_source(srcdir, builddir)

    # Upload tarball. Do not include versions in tarballs
    tarball_name = re.sub(r'[0-9]$','', args.package)

    # Trick to make mentor job project to get proper URLs
    if args.package == "mentor2":
        tarball_name = "mentor2"

    # For ignition, we use the alias without version numbers as package name
    if IGN_REPO:
        tarball_name = re.sub(r'[0-9]$','', args.package_alias)

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

    check_call(['s3cmd', 'sync', tarball_path, UPLOAD_DEST_PATTERN%get_canonical_package_name(args.package)])
    shutil.rmtree(tmpdir)

    # Tag repo
    os.chdir(sourcedir)

    try:
        tag = '%s_%s'%(args.package_alias, args.version)
        check_call(['hg', 'tag', '-f', tag])

        # Push tag
        check_call(['hg', 'push','-b','.'])
    except Exception as e:
        # Assume git
        pass

    source_tarball_uri = DOWNLOAD_URI_PATTERN%get_canonical_package_name(args.package) + tarball_fname

    ###################################################
    # Platform-specific stuff.
    # The goal is to build packages for specific platforms

    ###################################################
    # Ubuntu-specific stuff.
    # The goal is to build debs.

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
        repo_dir = download_release_repository(args.package, args.release_repo_branch)
        # The supported distros are the ones in the top level of -release repo
        ubuntu_distros = discover_ubuntu_distros(args, repo_dir)
        debian_distros = discover_debian_distros(args, repo_dir)
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

    if args.upload_to_repository in OSRF_REPOS_SELF_CONTAINED:
        params['OSRF_REPOS_TO_USE'] = args.upload_to_repository

    if NIGHTLY:
        params['VERSION'] = 'nightly'
        params['SOURCE_TARBALL_URI'] = ''

    if UPSTREAM:
        job_name = JOB_NAME_UPSTREAM_PATTERN%(args.package)
    else:
        job_name = JOB_NAME_PATTERN%(args.package)

    params_query = urllib.urlencode(params)

    # RELEASING FOR BREW
    brew_url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL,
                                                   GENERIC_BREW_PULLREQUEST_JOB,
                                                   params_query)
    print('- Brew: %s'%(brew_url))
    if not DRY_RUN and not NIGHTLY:
        urllib.urlopen(brew_url)

    # RELEASING FOR LINUX
    for l in LINUX_DISTROS:
        if (l == 'ubuntu'):
            distros = ubuntu_distros
        elif (l == 'debian'):
            if (PRERELEASE or NIGHTLY):
                continue
            distros = debian_distros
        else:
            error("Distro not supported in code")

        for d in distros:
            for a in SUPPORTED_ARCHS:
                # Filter prerelease and nightly architectures
                if (PRERELEASE or NIGHTLY):
                    if (a == 'armhf' or a == 'arm64'):
                        continue

                linux_platform_params = params.copy()
                linux_platform_params['ARCH'] = a
                linux_platform_params['LINUX_DISTRO'] = l
                linux_platform_params['DISTRO'] = d

                if (a == 'armhf' or a == 'arm64'):
                    # Need to use JENKINS_NODE_TAG parameter for large memory nodes
                    # since it runs qemu emulation
                    linux_platform_params['JENKINS_NODE_TAG'] = 'large-memory'

                if (NIGHTLY and a == 'i386'):
                    # only keep i386 for sdformat in nightly,
                    # just to test CI infrastructure
                    if (not args.package[:-1] == 'sdformat'):
                        continue

                linux_platform_params_query = urllib.urlencode(linux_platform_params)

                url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL, job_name, linux_platform_params_query)
                print('- Linux: %s'%(url))

                if not DRY_RUN:
                    urllib.urlopen(url)

if __name__ == '__main__':
    go(sys.argv)
