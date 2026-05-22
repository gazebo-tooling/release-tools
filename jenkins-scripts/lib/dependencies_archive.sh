#!/bin/bash

# **** WARNING ***** : when modifying this file
# **** WARNING ***** : any trailing whitespaces will break dependencies scapes

# mesa-utils, x11-utils for dri checks, xsltproc for qtest->junit conversion and
# python-psutil for memory testing
# netcat-openbsd (nc command) for squid-deb-proxy checking
# net-tools (route command) for squid-deb-proxy checking
# gnupg apt-key requires gnupg, gnupg2 or gnupg1
# xauth is needed inside the container so X11 client libs (libxcb in 26.04
# and newer) can read the host's MIT-MAGIC-COOKIE-1 cookie mounted via
# XAUTHORITY. See release-tools#1499.
BASE_DEPENDENCIES="build-essential \\
                   cmake           \\
                   debhelper       \\
                   mesa-utils      \\
                   x11-utils       \\
                   xauth           \\
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
