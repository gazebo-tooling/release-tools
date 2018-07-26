#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

export INSTALL_JOB_PKG="subt"
export INSTALL_JOB_REPOS="stable prerelease"
export USE_ROS_REPO=true

case "$DISTRO" in
  'bionic')
    export ROS_DISTRO=melodic
    ;;
  *)
    echo "Only bionic+melodic is supported"
    exit 1
    ;;
esac

export INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: testing by running lava_tube launch file'
. /opt/ros/$ROS_DISTRO/setup.bash
rosdep init
rosdep update


TEST_START=\`date +%s\`
timeout --preserve-status 180 roslaunch subt_gazebo lava_tube.launch || true
TEST_END=\`date +%s\`

DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

# Check the test run-time
if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
