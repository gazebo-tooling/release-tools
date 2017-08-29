#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="gazebo_ros_pkgs_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash ""

cat > build.sh << DELIM_CHECKOUT
###################################################
# Make project-specific changes here
#
set -ex

[[ -d ${WORKSPACE}/gazebo_ros_demos ]] && rm -fr ${WORKSPACE}/gazebo_ros_demos
git clone https://github.com/ros-simulation/gazebo_ros_demos ${WORKSPACE}/gazebo_ros_demos
DELIM_CHECKOUT

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "gazebo_ros_demos"
# SOFTWARE_DIR is set by _ros_setup_buildsh.bash. In this job we are getting 
# the gazebo_ros_demos in the build.sh so we need to avoid the variable to 
# reach the docker file generation (it will try to copy it to the container)
[[ -n ${SOFTWARE_DIR} ]] && unset SOFTWARE_DIR

cat >> build.sh << DELIM

TEST_TIMEOUT=90

TEST_START=\`date +%s\`
timeout --preserve-status \$TEST_TIMEOUT roslaunch gazebo_ros empty_world.launch verbose:=true || true
TEST_END=\`date +%s\`
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
   echo 'The test took less than \$TEST_TIMEOUT. Something bad happened'
   exit 1
fi

cd ${CATKIN_WS}/src/gazebo_ros_demos/
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PWD
cd ${CATKIN_WS}

TEST_TIMEOUT=180

TEST_START=\`date +%s\`
timeout --preserve-status \$TEST_TIMEOUT roslaunch rrbot_gazebo rrbot_world.launch headless:=true extra_gazebo_args:="--verbose"
TEST_END=\`date +%s\`
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
   echo 'The test took less than \$TEST_TIMEOUT. Something bad happened'
   exit 1
fi

echo "180 testing seconds finished successfully"
DELIM

USE_ROS_REPO=true

# To be sure about getting the latest versions of all dependencies:
# gazebo, sdformat, ignition ... we need to add them as packages in
# DEPENDENCY_PKGS. They would be automatically pulled by ROS_GAZEBO_PKGS
# but won't be updated if docker cache is in use. Adding them to the
# list will do it.
DEPENDENCY_PKGS="${ROS_CATKIN_BASE} git \
                 ${ROS_GAZEBO_PKGS} \
                 ${ROS_GAZEBO_PKGS_EXAMPLE_DEPS} \
                 ${_GZ_ROS_PACKAGES}"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
