#!/usr/bin/env python

from __future__ import print_function
import subprocess
import sys
import tempfile
import os
import urllib
import argparse
import shutil

USAGE = 'release.py <package> <version> <jenkinstoken>'
JENKINS_URL = 'http://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-debbuilder'
JOB_NAME_UPSTREAM_PATTERN = 'upstream-%s-debbuilder'
UPLOAD_DEST = 'ubuntu@gazebosim.org:/var/www/assets/distributions'
DOWNLOAD_URI = 'http://gazebosim.org/assets/distributions/'

UBUNTU_ARCHS = ['amd64', 'i386']
UBUNTU_DISTROS = ['precise']
UBUNTU_DISTROS_EXPERIMENTAL = ['trusty']

DRY_RUN = False
NIGHTLY = False
UPSTREAM = False
EXP_DISTROS = False

IGNORE_DRY_RUN = True

def error(msg):
    print("\n !! " + msg + "\n")
    sys.exit(1)

def print_success(msg):
    print("     + OK " + msg)

def parse_args(argv):
    global DRY_RUN
    global NIGHTLY
    global UPSTREAM
    global EXP_DISTROS

    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--nightly', dest='nightly', action='store_true', default=False,
                        help='Build nightly releases: do not upload tar.bz2 and values are autoconfigured')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    parser.add_argument('--no-sanity-checks', dest='no_sanity_checks', action='store_true', default=False,
                        help='no-sanity-checks; i.e. skip sanity checks commands')

    parser.add_argument('-e', '--experimental-distros', dest='exp_distros', action='store_true', default=False,
                        help='build packages for experimentally supported Ubuntu distros')
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
    args = parser.parse_args()
    if not args.package_alias:
        args.package_alias = args.package
    DRY_RUN = args.dry_run
    NIGHTLY = args.nightly
    UPSTREAM = args.upstream
    EXP_DISTROS = args.exp_distros
    return args

def get_release_repository_URL(package):
    return "https://bitbucket.org/osrf/" + package + "-release"

def download_release_repository(package, release_branch):
    url = get_release_repository_URL(package)
    release_tmp_dir = tempfile.mkdtemp() 
    cmd = [ "hg", "clone", "-b", release_branch, url, release_tmp_dir]
    check_call(cmd, IGNORE_DRY_RUN)
    return release_tmp_dir

def sanity_package_name(repo_dir, package, package_alias):
    expected_name = package

    if package_alias:
        expected_name = package_alias

    cmd = ["find", repo_dir, "-name", "changelog","-exec","head","-n","1","{}",";"]
    out, err = check_call(cmd)
    for line in out.split("\n"):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[0] != expected_name:
            error("Error in package name or alias: " + line)

    print_success("Package names in changelog")

def sanity_package_version(repo_dir, version, release_version):
    cmd = ["find", repo_dir, "-name", "changelog","-exec","head","-n","1","{}",";"]
    out, err = check_call(cmd)
    for line in out.split("\n"):
        if not line:
            continue
        # return full version in brackets
        full_version=line.split(' ')[1]
        # get only version (not release) in brackets
        c_version=full_version[full_version.find("(")+1:full_version.find("-")]
        c_revision=full_version[full_version.find("-")+1:full_version.find("~")]

        if c_version != version:
            error("Error in package version: " + full_version)

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

def sanity_checks(args):
    repo_dir = download_release_repository(args.package, args.release_repo_branch)
    sanity_package_name(repo_dir, args.package, args.package_alias)
    sanity_package_version(repo_dir, args.version, str(args.release_version))
    sanity_check_gazebo_versions(args.package, args.version)
    shutil.rmtree(repo_dir)

def check_call(cmd, ignore_dry_run = False):
    if NIGHTLY:
        return '',''
    if ignore_dry_run:
        # Commands that do not change anything in repo level
        print('Dry-run running:\n  %s'%(' '.join(cmd)))
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
            sys.exit(1)
        return out, err

def generate_upload_tarball(args):
    ###################################################
    # Platform-agnostic stuff.
    # The goal is to tag the repo and prepare a tarball.

    # Check for uncommitted changes; abort if present
    cmd = ['hg', 'status', '-q']
    out, err = check_call(cmd)
    if len(out) != 0:
        print('Mercurial says that you have uncommitted changes.  Please clean up your working copy so that "%s" outputs nothing'%(' '.join(cmd)))
        print('stdout: %s'%(out))
        sys.exit(1)

    # Tag repo
    tag = '%s_%s'%(args.package_alias, args.version)
    check_call(['hg', 'tag', '-f', tag])

    # Push tag
    check_call(['hg', 'push'])

    # Make a clean copy, to avoid pulling in other stuff that the user has
    # sitting in the working copy
    tmpdir = tempfile.mkdtemp() 
    srcdir = os.path.join(tmpdir, 'src')
    builddir = os.path.join(tmpdir, 'build')
    check_call(['hg', 'archive', srcdir])

    # configure and make package_source
    os.mkdir(builddir)
    os.chdir(builddir)
    check_call(['cmake', srcdir])
    check_call(['make', 'package_source'])

    # Upload tarball
    # We need to trick the current packages for the tarball
    tarball_name = args.package
    if args.package == "gazebo-current" or \
       args.package == "gazebo2" or \
       args.package == "gazebo3":
        tarball_name = "gazebo"
    # TODO: we're assuming a particular naming scheme and a particular compression tool
    tarball_fname = '%s-%s.tar.bz2'%(tarball_name, args.version)
    tarball_path = os.path.join(builddir, tarball_fname)
    # If we're releasing under a different name, then rename the tarball (the
    # package itself doesn't know anything about this).
    if args.package != args.package_alias:
        tarball_fname = '%s-%s.tar.bz2'%(args.package_alias, args.version)
        if (not args.dry_run) and (not args.nightly):
          shutil.copyfile(tarball_path, os.path.join(builddir, tarball_fname))
        tarball_path = os.path.join(builddir, tarball_fname)
    check_call(['scp', tarball_path, UPLOAD_DEST])
    shutil.rmtree(tmpdir)
    source_tarball_uri = DOWNLOAD_URI + tarball_fname

    ###################################################
    # Platform-specific stuff.
    # The goal is to build packages for specific platforms

    ###################################################
    # Ubuntu-specific stuff.
    # The goal is to build debs.

    # TODO: Consider auto-updating the Ubuntu changelog.  It requires
    # cloning the <package>-release repo, making a change, and pushing it back.
    # Until we do that, the user must have first updated it manually.
    if not args.nightly:
        print('\n\nReady to kick off the Ubuntu deb-builder.  Did you already update ubuntu/debian/changelog in the %s-release repo (on the %s branch)?  [y/N]'%(args.package, args. release_repo_branch))
        answer = sys.stdin.readline().strip()
        if answer != 'Y' and answer != 'y':
            print('Ubuntu deb-builds were NOT started.')
            print('Please update the changelog and try again.')
            sys.exit(1)

    return source_tarball_uri


def go(argv):
    args = parse_args(argv)

    # Default to release 1 if not present
    if not args.release_version:
        args.release_version = 1

    # Sanity checks before proceed. UPSTREAM repository is not known in release-tools script
    if not UPSTREAM and not args.no_sanity_checks:
        sanity_checks(args)

    source_tarball_uri = ''
    if not UPSTREAM:
        source_tarball_uri = generate_upload_tarball(args)
    
    # Kick off Jenkins jobs
    params = {}
    params['token'] = args.jenkins_token
    params['PACKAGE'] = args.package
    params['VERSION'] = args.version
    params['SOURCE_TARBALL_URI'] = source_tarball_uri
    params['RELEASE_REPO_BRANCH'] = args.release_repo_branch
    params['PACKAGE_ALIAS'] = args.package_alias
    params['RELEASE_VERSION'] = args.release_version
    if NIGHTLY:
        params['VERSION'] = 'nightly'
        params['SOURCE_TARBALL_URI'] = ''
        params['RELEASE_REPO_BRANCH'] = 'nightly'

    if UPSTREAM:
        job_name = JOB_NAME_UPSTREAM_PATTERN%(args.package)
    else:
        job_name = JOB_NAME_PATTERN%(args.package)
    params_query = urllib.urlencode(params)
    base_url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL, job_name, params_query)
    distros = UBUNTU_DISTROS
    if EXP_DISTROS:
        distros.extend(UBUNTU_DISTROS_EXPERIMENTAL)
    for d in distros:
        for a in UBUNTU_ARCHS:
            url = '%s&ARCH=%s&DISTRO=%s'%(base_url, a, d)
            print('Accessing: %s'%(url))
            if not DRY_RUN:
                urllib.urlopen(url)

    ###################################################
    # Fedora-specific stuff.
    # The goal is to build rpms.
    # TODO

if __name__ == '__main__':
    go(sys.argv)
