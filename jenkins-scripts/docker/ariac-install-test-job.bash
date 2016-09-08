#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

export INSTALL_JOB_PKG="ariac"
export INSTALL_JOB_REPOS="stable"
export USE_ROS_REPO=true

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: testing by running ariac launch file'
. /opt/ros/indigo/setup.bash
rosdep init
rosdep update

TEST_START=\`date +%s\`
timeout --preserve-status 180 roslaunch osrf_gear ur10_example_world.launch || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
