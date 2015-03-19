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
UPLOAD_DEST_PATTERN = 's3://osrf-distributions/%s/releases/'
DOWNLOAD_URI_PATTERN = 'http://gazebosim.org/distributions/%s/releases/'

UBUNTU_ARCHS = ['amd64', 'i386']
UBUNTU_DISTROS = ['utopic', 'trusty']
UBUNTU_DISTROS_EXPERIMENTAL = []

ROS_DISTROS_IN_PRECISE = [ 'hydro' ]
ROS_DISTROS_IN_TRUSTY = [ 'indigo' ];

DRY_RUN = False
NIGHTLY = False
UPSTREAM = False
NO_SRC_FILE = False
EXP_DISTROS = False
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

def parse_args(argv):
    global DRY_RUN
    global NIGHTLY
    global UPSTREAM
    global NO_SRC_FILE
    global EXP_DISTROS
    global DRCSIM_MULTIROS
    global IGN_REPO

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
                        help='no-sanity-checks; i.e. skip sanity checks commands')
    parser.add_argument('--ignition-repo', dest='ignition_repo', action='store_true', default=False,
                        help='use ignition robotics URL repositories instead of OSRF')

    args = parser.parse_args()
    if not args.package_alias:
        args.package_alias = args.package
    DRY_RUN = args.dry_run
    NIGHTLY = args.nightly
    UPSTREAM = args.upstream
    NO_SRC_FILE = args.no_source_file
    EXP_DISTROS = args.exp_distros
    DRCSIM_MULTIROS = args.drcsim_multiros
    IGN_REPO = args.ignition_repo

    # Upstream and nightly do not generate a tar.bz2 file
    if args.upstream or args.nightly:
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
    out, err = check_call(cmd)
    for line in out.split("\n"):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[0] != expected_name:
            error("Error in changelog package name or alias: " + line)

    cmd = ["find", repo_dir, "-name", "control","-exec","grep","-H","Source:","{}",";"]
    out, err = check_call(cmd)
    for line in out.split("\n"):
        if not line:
            continue
        # Check that first word is the package alias or name
        if line.partition(' ')[2] != expected_name:
            error("Error in source package. File:  " + line.partition(' ')[1] + ". Got " + line.partition(' ')[2] + " expected " + expected_name)

    print_success("Package names in changelog and control")

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

def sanity_checks(args):
    check_s3cmd_configuration()

    repo_dir = download_release_repository(args.package, args.release_repo_branch)
    sanity_package_name_underscore(args.package, args.package_alias)
    sanity_package_name(repo_dir, args.package, args.package_alias)
    sanity_package_version(repo_dir, args.version, str(args.release_version))
    sanity_check_gazebo_versions(args.package, args.version)
    sanity_check_sdformat_versions(args.package, args.version)
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
            raise Exception('subprocess call failed')
        return out, err

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

    # configure and make package_source
    os.mkdir(builddir)
    os.chdir(builddir)
    check_call(['cmake', srcdir])
    check_call(['make', 'package_source'])

    # Upload tarball. Do not include versions in tarballs
    tarball_name = re.sub(r'[0-9]$','', args.package)
    # We need to trick the gazebo-current (version 2)
    if args.package == "gazebo-current":
        tarball_name = "gazebo"
    # For ignition, we use the alias as package name
    if IGN_REPO:
        tarball_name = args.package_alias

    # TODO: we're assuming a particular naming scheme and a particular compression tool
    tarball_fname = '%s-%s.tar.bz2'%(tarball_name, args.version)
    tarball_path = os.path.join(builddir, tarball_fname)
    # If we're releasing under a different name, then rename the tarball (the
    # package itself doesn't know anything about this).
    if args.package != args.package_alias:
        tarball_fname = '%s-%s.tar.bz2'%(args.package_alias, args.version)
        if (not args.dry_run):
            if not os.path.isfile(tarball_path):
                error("Failed to found the tarball: " + tarball_path + 
                      ". Please check that you don't have an underscore in the project() statement of the CMakeList.txt. In that case, change it by a dash")
            dest_file = os.path.join(builddir, tarball_fname)
            # Do not copy if files are the same
            if not (tarball_path == dest_file):
                shutil.copyfile(tarball_path, dest_file)
                tarball_path = dest_file

    check_call(['s3cmd', 'put', tarball_path, UPLOAD_DEST_PATTERN%get_canonical_package_name(args.package)])
    shutil.rmtree(tmpdir)

    # Tag repo
    os.chdir(sourcedir)

    try:
        tag = '%s_%s'%(args.package_alias, args.version)
        check_call(['hg', 'tag', '-f', tag])
 
        # Push tag
        check_call(['hg', 'push'])
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

    # Do not generate source file if not needed or impossible
    if not args.no_source_file:
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
    
    # Enable new armhf jobs in sdformat2 and gazebo5
    if (args.package == 'sdformat2' or args.package == 'gazebo5'):
        # No prereleases
        if (not args.release_repo_branch == 'prerelease'):
            UBUNTU_ARCHS.append('armhf')
   
    params_query = urllib.urlencode(params)
    distros = UBUNTU_DISTROS
    if EXP_DISTROS:
        distros.extend(UBUNTU_DISTROS_EXPERIMENTAL)

    for d in distros:
        for a in UBUNTU_ARCHS:
            if (a == 'armhf'):
                # Only release armhf in trusty for now
                if (d != 'trusty'):
                    continue
                # armhf runs on docker, it needs a different base_url
                base_url = '%s/job/%s-docker/buildWithParameters?%s'%(JENKINS_URL, job_name, params_query)
            else:
                base_url = '%s/job/%s/buildWithParameters?%s'%(JENKINS_URL, job_name, params_query)

            if not DRCSIM_MULTIROS:
                url = '%s&ARCH=%s&DISTRO=%s'%(base_url, a, d)
                print('Accessing: %s'%(url))

                if not DRY_RUN:
                    urllib.urlopen(url)
            else:
                if (d == 'precise'):
                    ROS_DISTROS = ROS_DISTROS_IN_PRECISE
                elif (d == 'trusty'):
                    ROS_DISTROS = ROS_DISTROS_IN_TRUSTY
                else:
                    print ("ERROR in ROS_DISTROS: unkonwn distribution")
                    sys.exit(1)

                for r in ROS_DISTROS:
                    url = '%s&ARCH=%s&DISTRO=%s&ROS_DISTRO=%s'%(base_url, a, d, r)
                    print('Accessing multiros: %s'%(url))
                    if not DRY_RUN:
                        urllib.urlopen(url)
        
    ###################################################
    # Fedora-specific stuff.
    # The goal is to build rpms.
    # TODO

if __name__ == '__main__':
    go(sys.argv)
