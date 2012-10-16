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
UPLOAD_DEST = 'ubuntu@gazebosim.org:/tmp/tarballs'

UBUNTU_ARCHS = ['i386', 'amd64']
UBUNTU_DISTROS = ['precise']

DRY_RUN = False

def parse_args(argv):
    global DRY_RUN
    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    args = parser.parse_args()
    DRY_RUN = args.dry_run
    return args

def check_call(cmd):
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
    cmd = ['hg', 'status']
    out, err = check_call(cmd)
    if len(out) != 0:
        print('Mercurial says that you have uncommitted changes.  Please clean up your working copy so that "%s" outputs nothing'%(' '.join(cmd)))
        print('stdout: %s'%(out))
        sys.exit(1)

    # Tag repo
    tag = '%s_%s'%(args.package, args.version)
    check_call(['hg', 'tag', '-f', tag])

    # Push tag
    check_call(['hg', 'push'])

    # Make build tmpdir, configure and make package_source
    tmpdir = tempfile.mkdtemp() 
    os.chdir(tmpdir)
    check_call(['cmake', pwd])
    check_call(['make', 'package_source'])

    # Upload tarball
    # TODO: we're assuming a particular naming scheme and a particular compression tool
    tarball_fname = '%s/%s-%s.tar.bz2'%(tmpdir, args.package, args.version)
    check_call(['scp', tarball_fname, UPLOAD_DEST])
    shutil.rmtree(tmpdir)

    ###################################################
    # Platform-specific stuff.
    # The goal is to build packages for specific platforms

    ###################################################
    # Ubuntu-specific stuff.
    # The goal is to build debs.

    # TODO: Consider auto-updating the Ubuntu changelog.  It requires
    # cloning the <package>-release repo, making a change, and pushing it back.
    # Until we do that, the user must have first updated it manually.
    print('\n\nReady to kick off the Ubuntu deb-builder.  Did you already update ubuntu/debian/changelog in the %s-release repo? [y/N]'%(args.package))
    answer = sys.stdin.readline().strip()
    if answer != 'Y' and answer != 'y':
        print('Ubuntu deb-builds were NOT started.')
        print('Please update the changelog and try again.')
        sys.exit(1)

    # Kick off Jenkins jobs
    base_url = '%s/job/%s/buildWithParameters?token=%s&PACKAGE=%s&VERSION=%s&TAG=%s'%(JENKINS_URL, JOB_NAME_PATTERN%(args.package), args.jenkins_token, args.package, args.version, tag)
    for d in UBUNTU_DISTROS:
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
