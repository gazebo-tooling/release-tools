#!/usr/bin/env python

from __future__ import print_function

import pkg_resources
import em
import os


def expand_template(config_template, d):
    s = em.expand(config_template, **d)
    return s

def setup_directories(rootdir):
    """ Create the directories needed to use apt with an alternate
    rootdir """

    # create the directories needed
    dirs = ["etc/apt/sources.list.d", 
            "etc/apt/apt.conf.d",
            "etc/apt/preferences.d",
            "var/lib/apt/lists/partial",
            "var/cache/apt/archives/partial",
            "var/lib/dpkg"
            ]
    
    for d in dirs:
        try:
            os.makedirs(os.path.join(rootdir, d))
        except OSError, ex:
            if ex.errno == 17:
                continue
            raise ex

def setup_conf(rootdir, target_dir):
    """ Set the apt.conf config settings for the specific
    architecture. """

    d = {'rootdir':rootdir}
    with open(os.path.join(target_dir, "apt.conf"), 'w') as apt_conf:
        template = pkg_resources.resource_string('buildfarm', 'resources/templates/apt.conf.em')
        apt_conf.write(expand_template(template, d))


    
def set_default_sources(rootdir, distro, repo):
    """ Set the source lists for the default ubuntu and ros sources """
    d = {'distro':distro, 
         'repo': repo}
    with open(os.path.join(rootdir, "etc/apt/sources.list"), 'w') as sources_list:
        template = pkg_resources.resource_string('buildfarm', 'resources/templates/sources.list.em')
        sources_list.write(expand_template(template, d))

def set_additional_sources(rootdir, distro, repo, source_name):
    """ Set the source lists for the default ubuntu and ros sources """
    d = {'distro':distro, 
         'repo': repo}
    with open(os.path.join(rootdir, "etc/apt/sources.list.d/%s.list"%source_name), 'w') as sources_list:
        template = pkg_resources.resource_string('buildfarm', 'resources/templates/ros-sources.list.em')
        sources_list.write(expand_template(template, d))
    

def setup_apt_rootdir(rootdir, distro, arch, mirror=None, additional_repos = {}):
    setup_directories(rootdir)
    if not mirror:
        repo='http://us.archive.ubuntu.com/ubuntu/'
    else:
        repo = mirror
    set_default_sources(rootdir, distro, repo)
    for repo_name, repo_url in additional_repos.iteritems():
        set_additional_sources(rootdir, distro, repo_url, repo_name)

    d = {'arch':arch}
    path = os.path.join(rootdir, "etc/apt/apt.conf.d/51Architecture")
    with open(path, 'w') as arch_conf:
        template = pkg_resources.resource_string('buildfarm', 'resources/templates/arch.conf.em')
        arch_conf.write(expand_template(template, d))


def parse_repo_args(repo_args):
    """ Split the repo argument listed as "repo_name@repo_url" into a map"""
    ros_repos = {}
    
    for a in repo_args:
        n, u = a.split('@')
        ros_repos[n] = u

    return ros_repos

