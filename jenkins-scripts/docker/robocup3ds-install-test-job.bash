#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export INSTALL_JOB_PKG="gazebo6-robocup3ds"
export INSTALL_JOB_REPOS="stable prerelease"

INSTALL_JOB_PREINSTALL_HOOK="""
cat >> /var/cache/debconf/config.dat << DELIM
Name: robocup3ds-nao-meshes/accepted-robocup3ds-nao-meshes
Template: robocup3ds-nao-meshes/accepted-robocup3ds-nao-meshes
Value: true
Owners: gazebo6-robocup3ds, libgazebo6-robocup3ds
Flags: seen
DELIM
"""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
