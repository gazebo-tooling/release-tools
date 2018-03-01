#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

INSTALL_JOB_PREINSTALL_HOOK="""
# import the SRC repo
echo \"deb http://srcsim.gazebosim.org/src ${DISTRO} main\" >\\
                                           /etc/apt/sources.list.d/src.list
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
wget -qO - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add - 
sudo apt-get update
"""

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: testing by running ihmc_valkyrie_ros launch file'
mkdir ~/valkyrie
. /opt/ros/indigo/setup.bash
. /opt/nasa/indigo/setup.bash

TEST_START=\`date +%s\`
timeout --preserve-status 180 roslaunch ihmc_valkyrie_ros ihmc_valkyrie_scs.launch extra_gazebo_args:=\"--verbose\" || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""
# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="wget bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
