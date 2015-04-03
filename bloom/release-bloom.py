#!/usr/bin/env python

from __future__ import print_function
import subprocess
import sys
import tempfile
import os
import urllib
import argparse
import shutil

USAGE = 'release.py <package> <version> <upstream_release_repo> <jenkinstoken>'
JENKINS_URL = 'http://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-bloom-debbuilder'

UBUNTU_ARCHS = ['amd64']
# UBUNTU_DISTROS = ['trusty', 'precise']
UBUNTU_DISTROS = ['trusty']
ROS_DISTROS_IN_PRECISE = [ 'hydro' ]
ROS_DISTROS_IN_TRUSTY = [ 'indigo' ]

DRY_RUN = False

def parse_args(argv):
    global DRY_RUN

    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('upstream_release_repo', help='URL of -release upstream repository')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    parser.add_argument('-r', '--release-version', dest='release_version', 
                        default=None,
                        help='Release version suffix; usually 1 (e.g., 1')
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

    # Check that package does not have an underscore (not allowed by policy)
    if (args.package.find("_") > -1):
        print('Error: package name can not have an underscore (you probably want a hyphen)')
        sys.exit(1)

    # Kick off Jenkins jobs
    params = {}
    params['token'] = args.jenkins_token
    params['PACKAGE'] = args.package
    params['VERSION'] = args.version
    params['UPSTREAM_RELEASE_REPO'] = args.upstream_release_repo

    if not args.release_version:
        args.release_version = 0
    params['RELEASE_VERSION'] = args.release_version
    params_query = urllib.urlencode(params)
    base_url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL, JOB_NAME_PATTERN%(args.package), params_query)
    distros = UBUNTU_DISTROS
    for d in distros:
        for a in UBUNTU_ARCHS:
            # Process ROS distros for each ubuntu distro
            # raring           -> hydro
            # quantal, precise -> groovy + hydro
            if (d == 'precise'):
                ROS_DISTROS = ROS_DISTROS_IN_PRECISE
            elif (d == 'trusty'):
                ROS_DISTROS = ROS_DISTROS_IN_TRUSTY
            else:
                print ("Unkwnon distribution")
                sys.exit(1)

            for r in ROS_DISTROS:
                url = '%s&ARCH=%s&DISTRO=%s&ROS_DISTRO=%s'%(base_url, a, d, r)
                print('Accessing: %s'%(url))
                if not DRY_RUN:
                    urllib.urlopen(url)

if __name__ == '__main__':
    go(sys.argv)
