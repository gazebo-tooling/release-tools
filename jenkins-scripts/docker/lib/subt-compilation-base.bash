#!/bin/bash -x

case ${DISTRO} in
  'kinetic')
    ROS_DISTRO=melodic
    GAZEBO_VERSION_FOR_ROS="9"
    ;;
  'bionic')
    # 9 is the default version in Bionic
    ROS_DISTRO=melodic
    USE_DEFAULT_GAZEBO_VERSION_FOR_ROS=true
    ;;
  *)
    echo "Unsupported DISTRO: ${DISTRO}"
    exit 1
esac

export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="subt_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh


export ROS_SETUP_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: smoke test'
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
mkdir -p ~/.gazebo/models
tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz

pwd
find . -name setup.bash

# Test installation
roslaunch subt_gazebo lava_tube.launch || true

catkin env 'timeout --preserve-status 180 roslaunch subt_gazebo lava_tube.launch extra_gazebo_args:=verbose' || true

TEST_START=\$(date +%s)
timeout --preserve-status \$TEST_TIMEOUT roslaunch servicesim servicesim.launch headless:=true extra_gazebo_args:=\"--verbose\"
TEST_END=\$(date +%s)
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
  echo \"The test took less than \$TEST_TIMEOUT. Something bad happened.\"
  exit 1
fi

echo 'Smoke testing completed successfully.'
echo '# END SECTION'
"""

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "subt"

DEPENDENCY_PKGS="${SUBT_DEPENDENCIES}"
# ROS packages come from the mirror in the own subt repository
USE_ROS_REPO=true
OSRF_REPOS_TO_USE="stable"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
