#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="drake_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash ""

cat > build.sh << DELIM_CHECKOUT
###################################################
# Make project-specific changes here
#
set -ex

# Temporary branch in osrf-rosdep
rosdep init
echo "yaml https://github.com/osrf/osrf-rosdep/raw/rosdep_drake/drake/drake.yaml" > /etc/ros/rosdep/sources.list.d/00-drake.list
echo "yaml https://github.com/osrf/osrf-rosdep/raw/rosdep_drake/drake/releases/kinetic.yaml $ROS_DISTRO" >> /etc/ros/rosdep/sources.list.d/00-drake.list

[[ -d ${WORKSPACE}/kumonoito ]] && rm -fr ${WORKSPACE}/kumonoito
git clone https://github.com/osrf/kumonoito -b use_ros_drake ${WORKSPACE}/kumonoito

# Need to use clang to match ABI
update-alternatives --set c++ /usr/bin/clang++
update-alternatives --set cc /usr/bin/clang
DELIM_CHECKOUT

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "kumonoito"
# SOFTWARE_DIR is set by _ros_setup_buildsh.bash. In this job we are getting 
# the kumonoito in the build.sh so we need to avoid the variable to 
# reach the docker file generation (it will try to copy it to the container)
[[ -n ${SOFTWARE_DIR} ]] && unset SOFTWARE_DIR

cat >> build.sh << DELIM
cd ${CATKIN_WS}/src/kumonoito/
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PWD
cd ${CATKIN_WS}

# TEST_TIMEOUT=180

# TEST_START=\`date +%s\`
# timeout --preserve-status \$TEST_TIMEOUT roslaunch rrbot_gazebo rrbot_world.launch headless:=true extra_gazebo_args:="--verbose"
# TEST_END=\`date +%s\`
# DIFF=\$(expr \$TEST_END - \$TEST_START)
#
# if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
#   echo 'The test took less than \$TEST_TIMEOUT. Something bad happened'
#   exit 1
# fi

# echo "180 testing seconds finished successfully"
DELIM

USE_ROS_REPO=true
OSRF_REPOS_TO_USE="drake"
# Clang is needed to be able to have the same CXX11 ABI produced in the drake package
DEPENDENCY_PKGS="${ROS_CATKIN_BASE} git ros-kinetic-ros-drake clang"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
