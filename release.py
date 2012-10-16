#!/usr/bin/env python

from __future__ import print_function
import subprocess
import sys
import tempfile
import os
import urllib

USAGE = 'release.py <package> <version> <jenkinstoken>'
JENKINS_URL = 'http://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-debbuild'

UBUNTU_ARCHS = ['i386', 'amd64']
UBUNTU_DISTROS = ['precise']

def parse_args(argv):
    if len(argv) != 4:
        print(USAGE)
        sys.exit(1)
    return (argv[1], argv[2], argv[3])

def check_call(cmd):
    print('Running:\n  %s'%(' '.join(cmd)))
    po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = po.communicate()
    if po.returncode != 0:
        print('Error running Mercurial (%s).  Is the current working directory a Mercurial repo? (it should be)'%(' '.join(cmd)))
        print('stdout: %s'%(out))
        print('stderr: %s'%(err))
        sys.exit(1)
    return out, err

def go(argv):
    package, version, jenkins_token = parse_args(argv)

    pwd = os.getcwd()

    # Check for uncommitted changes; abort if present
    cmd = ['hg', 'status']
    out, err = check_call(cmd)
    if len(out) != 0:
        print('Mercurial says that you have uncommitted changes.  Please clean up your working copy so that "%s" outputs nothing'%(' '.join(cmd)))
        print('stdout: %s'%(out))
        sys.exit(1)

    # Tag repo
    tag = '%s_%s'%(package, version)
    check_call(['hg', 'tag', '-f', tag])

    # Push tag
    check_call(['hg', 'push'])

    # Make build tmpdir, configure and make package_source
    tmpdir = tempfile.mkdtemp() 
    os.chdir(tmpdir)
    check_call(['cmake', pwd])
    check_call(['make', 'package_source'])

    # TODO: Upload tarball

    # Platform-specific stuff:

    # TODO: Update Ubuntu changelog

    # Kick off Jenkins jobs
    base_url = '%s/job/%s/buildWithParameters?token=%s&PACKAGE=%s&VERSION=%s&TAG=%s'%(JENKINS_URL, JOB_NAME_PATTERN%(package), jenkins_token, package, version, tag)
    for d in UBUNTU_DISTROS:
        for a in UBUNTU_ARCHS:
            url = '%s&ARCH=%s&DISTRO=%s'%(base_url, a, d)
            print('Accessing: %s'%(url))
            urllib.urlopen(url)

if __name__ == '__main__':
    go(sys.argv)
