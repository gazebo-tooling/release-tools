#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes

# mesa-utils, x11-utils for dri checks, xsltproc for qtest->junit conversion and
# python-psutil for memory testing
# netcat-openbsd (nc command) for squid-deb-proxy checking
# net-tools (route command) for squid-deb-proxy checking
# gnupg apt-key requires gnupg, gnupg2 or gnupg1
BASE_DEPENDENCIES="build-essential \\
                   cmake           \\
                   debhelper       \\
                   mesa-utils      \\
                   x11-utils       \\
                   cppcheck        \\
                   xsltproc        \\
                   python3-lxml    \\
                   python3-psutil  \\
                   python3         \\
                   bc              \\
                   netcat-openbsd  \\
                   gnupg2          \\
                   net-tools       \\
                   locales         \\
                   sudo"

BREW_BASE_DEPENDCIES="git cmake"

if [[ -z $ROS_DISTRO ]]; then
  echo "------------------------------------------------------------"
  echo "ROS_DISTRO was not set before using dependencies_archive.sh!"
  echo "skipping ROS related variables"
  echo "------------------------------------------------------------"
else
  # colcon has no python2 candidate
  if ${ROS2}; then
    ROS_CATKIN_BASE="python3-dev                      \\
                    python3-colcon-common-extensions     \\
                    python3-rosdep                    \\
                    python3-wstool                    \\
                    python3-rosinstall                \\
                    python3-rospkg                    \\
                    python3-vcstools"
  fi
fi
