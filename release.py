#!/usr/bin/env python

from __future__ import print_function
import subprocess
import sys
import tempfile
import os

USAGE = 'release.py <package> <version>'

def parse_args(argv):
    if len(argv) != 3:
        print(USAGE)
    return (argv[1], argv[2])

def check_call(cmd):
    print 'Running:\n  %s'%(' '.join(cmd))
    po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = po.communicate()
    if po.returncode != 0:
        print('Error running Mercurial (%s).  Is the current working directory a Mercurial repo? (it should be)'%(' '.join(cmd)))
        print('stdout: %s'%(out))
        print('stderr: %s'%(err))
        sys.exit(1)
    return out, err

def go(argv):
    package, version = parse_args(argv)

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
    check_call(['hg', 'tag', tag])

    # Push tag
    check_call(c['hg', 'push'])

    # Make build tmpdir, configure and make package_source
    tmpdir = tempfile.mkdtemp() 
    os.chdir(tmpdir)
    check_call(['cmake', pwd])
    check_call(['make', 'package_source'])

    # Upload tarball: TODO
    check_call(['scp'])

if __name__ == '__main__':
    go(sys.argv)
