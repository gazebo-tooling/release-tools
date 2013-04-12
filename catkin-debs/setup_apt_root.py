#!/usr/bin/env python

from __future__ import print_function
import argparse

import buildfarm.apt_root

def parse_options():
    parser = argparse.ArgumentParser(
             description='setup a directory to be used as a rootdir for apt')
    parser.add_argument('--mirror', dest='mirror', action='store', default='http://us.archive.ubuntu.com/ubuntu/',
           help='The url for the default repo, like --mirror to debootstrap')
    parser.add_argument('--repo', dest='repo_urls', action='append',metavar=['REPO_NAME@REPO_URL'],
           help='The name for the source and the url such as ros@http://50.28.27.175/repos/building')
    parser.add_argument(dest='distro',
           help='The debian release distro, lucid, oneiric, etc')
    parser.add_argument(dest='architecture',
           help='The debian binary architecture. amd64, i386, armel')
    parser.add_argument(dest='rootdir',
           help='The rootdir to use')
    parser.add_argument('--local-conf-dir',dest='local_conf',
                      help='A directory to write an apt-conf to use with apt-get update.')
    args = parser.parse_args()


    if not args.repo_urls:
        #default to devel machine for now
        args.repo_urls = ['ros@http://50.28.27.175/repos/building']

    for a in args.repo_urls:
        if not '@' in a:
            parser.error("Invalid repo definition: %s"%a)


    return args


def doit():
    args = parse_options()

    ros_repos = buildfarm.apt_root.parse_repo_args(args.repo_urls)

    buildfarm.apt_root.setup_apt_rootdir(args.rootdir, args.distro, args.architecture, mirror= args.mirror, additional_repos = ros_repos)
    if args.local_conf:
        buildfarm.apt_root.setup_conf(args.rootdir, args.local_conf)


if __name__ == "__main__":
    doit()
