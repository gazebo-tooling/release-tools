#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG=""
export INSTALL_JOB_REPOS=""

INSTALL_JOB_PREINSTALL_HOOK="""
cat >> /var/cache/debconf/config.dat << DELIM
Name: robocup3ds-nao-meshes/accepted-robocup3ds-nao-meshes
Template: robocup3ds-nao-meshes/accepted-robocup3ds-nao-meshes
Value: true
Owners: gazebo6-robocup3ds, libgazebo6-robocup3ds
Flags: seen
DELIM
# Need to install libgnome2-common to complete the the workaround
# since the script will check for a interactive debconf (gnome/kde)
apt-get install -y libgnome2-common
"""

INSTALL_JOB_POSTINSTALL_HOOK="""
curl -ssL https://bitbucket.org/osrf/release-tools/raw/default/one-line-installations/robocup3ds.sh | sh
"""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
