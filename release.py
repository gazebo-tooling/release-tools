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
JOB_NAME_PATTERN = '%s-debbuild'
UPLOAD_DEST = 'ubuntu@gazebosim.org:/var/www/assets/distributions'
DOWNLOAD_URI = 'http://gazebosim.org/assets/distributions/'

UBUNTU_ARCHS = ['amd64', 'i386']
UBUNTU_DISTROS = ['precise','quantal']
UBUNTU_DISTROS_EXPERIMENTAL = ['raring']

DRY_RUN = False
NIGHTLY = False
EXP_DISTROS = False

def parse_args(argv):
    global DRY_RUN
    global NIGHTLY
    global EXP_DISTROS

    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--nightly', dest='nightly', action='store_true', default=False,
                        help='Build nightly releases: do not upload tar.bz2 and values are autoconfigured')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    parser.add_argument('-e', '--experimental-distros', dest='exp_distros', action='store_true', default=False,
                        help='build packages for experimentally supported Ubuntu distros')
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
    EXP_DISTROS = args.exp_distros
    return args

def check_call(cmd):
    if NIGHTLY:
        return '',''
    print('Running:\n  %s'%(' '.join(cmd)))
    if DRY_RUN:
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

def go(argv):
    args = parse_args(argv)

    pwd = os.getcwd()

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
    # TODO: we're assuming a particular naming scheme and a particular compression tool
    tarball_fname = '%s-%s.tar.bz2'%(args.package, args.version)
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

    # Kick off Jenkins jobs
    #TODO: remove TAG arg after all debbuild jobs are updated to look at SOURCE_TARBALL_URI
    params = {}
    params['token'] = args.jenkins_token
    params['PACKAGE'] = args.package
    params['VERSION'] = args.version
    params['SOURCE_TARBALL_URI'] = source_tarball_uri
    params['RELEASE_REPO_BRANCH'] = args.release_repo_branch
    params['PACKAGE_ALIAS'] = args.package_alias
    params['TAG'] = tag
    if NIGHTLY:
        params['VERSION'] = 'nightly'
        params['SOURCE_TARBALL_URI'] = ''
        params['RELEASE_REPO_BRANCH'] = 'nightly'

    if not args.release_version:
        args.release_version = 1
    params['RELEASE_VERSION'] = args.release_version
    params_query = urllib.urlencode(params)
    base_url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL, JOB_NAME_PATTERN%(args.package), params_query)
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
