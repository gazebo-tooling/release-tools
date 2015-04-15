LIB SCRIPTS LIST
======================

 1. generic-repo-release
 1. debian-git-repo-base

1. generic-repo-release.bash
----------------------------

! Not using git-buildpackage format and tags

This script is to use with a git repo which contains the the source and the
debian directory already in place. Plain git + debian.

 - Checkout is done in jenkins
 - Use version in the changelog
 + Change changelog distro  -> $DISTRO

Job to use:

 - release-tools/jenkins-scripts/generic-repo-debbuild.bash 

Example jobs:

 - http://build.osrfoundation.org/job/upstream-ogre3d-debbuilder/

1. debian-git-repo-base.bash
----------------------------

This script is using a debian-like git repo to generate a custom package
from it. Added an ~osrf postfix to version and modify the DISTRO
 
 + Checkout branch/tag at $BRANCH
 + Change changelog distro  -> $DISTRO
 + Change changelog version -> changelog_version + osrf$RELEASE_VERSION
                                                 + ~$DISTRO 
                                                 + $RELEASE_ARCH_VERSION

Job to use:

 - release-tools/jenkins-scripts/debian-git-debbuild.bash

Example jobs:

 - http://build.osrfoundation.org/job/zeromq-debian-debbuilder

TODO:

 - The script is not using git-buildpackage which could be an improvement
