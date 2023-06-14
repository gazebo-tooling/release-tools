#!/usr/bin/env python3

from __future__ import print_function
import argparse
import subprocess
import sys
import urllib.request
import urllib.parse

USAGE = 'release-bloom.py <package> <version> <upstream_release_repo> <ros_distro> <jenkinstoken>'
JENKINS_URL = 'http://build.osrfoundation.org'
JOB_NAME_PATTERN = '%s-bloom-debbuilder'

UBUNTU_ARCHS = ['amd64']
UBUNTU_DISTROS_IN_ROS = {'noetic': ['focal']}
UBUNTU_DISTROS_IN_ROS2 = {'foxy': ['focal'],
                          'humble': ['jammy'],
                          'iron': ['jammy'],
                          'rolling': ['jammy']}
DRY_RUN = False

def parse_args(argv):
    global DRY_RUN

    parser = argparse.ArgumentParser(description='Make releases.')
    parser.add_argument('package', help='which package to release')
    parser.add_argument('version', help='which version to release')
    parser.add_argument('upstream_release_repo', help='URL of -release upstream repository')
    parser.add_argument('ros_distro', help='ROS release to build packages to')
    parser.add_argument('jenkins_token', help='secret token to allow access to Jenkins to start builds')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true', default=False,
                        help='dry-run; i.e., do actually run any of the commands')
    parser.add_argument('-r', '--release-version', dest='release_version',
                        default=None,
                        help='Release version suffix; usually 1 (e.g., 1')
    parser.add_argument('--upload-to-repo', dest='upload_to_repository', default="stable",
                        help='OSRF repo to upload: stable | prerelease | nightly')
    args = parser.parse_args()
    DRY_RUN = args.dry_run
    return args


def check_call(cmd):
    print('Running:\n  %s' % (' '.join(cmd)))
    if DRY_RUN:
        return '', ''
    else:
        po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = po.communicate()
        if po.returncode != 0:
            print('Error running command (%s).' % (' '.join(cmd)))
            print('stdout: %s' % (out))
            print('stderr: %s' % (err))
            sys.exit(1)
        return out, err


def call_jenkins_jobs(argv):
    args = parse_args(argv)
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
    params['UPLOAD_TO_REPO'] = args.upload_to_repository

    ubuntu_per_ros_distro = UBUNTU_DISTROS_IN_ROS

    if args.ros_distro in UBUNTU_DISTROS_IN_ROS2:
        params['ROS2'] = True
        ubuntu_per_ros_distro = UBUNTU_DISTROS_IN_ROS2

    if not args.release_version:
        args.release_version = 0
    params['RELEASE_VERSION'] = args.release_version
    params_query = urllib.parse.urlencode(params)
    base_url = '%s/job/%s/buildWithParameters?%s' % \
               (JENKINS_URL, JOB_NAME_PATTERN % (args.package), params_query)

    for ubuntu_distro in ubuntu_per_ros_distro[args.ros_distro]:
        for arch in UBUNTU_ARCHS:
            url = '%s&ARCH=%s&DISTRO=%s&ROS_DISTRO=%s' % \
                  (base_url, arch, ubuntu_distro, args.ros_distro)
            print('Accessing: %s' % (url))
            if not DRY_RUN:
                urllib.request.urlopen(url)


if __name__ == '__main__':
    call_jenkins_jobs(sys.argv)
