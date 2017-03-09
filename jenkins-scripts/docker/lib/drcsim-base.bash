#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="drcsim_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Generate the first part of the build.sh file for ROS
. ${SCRIPT_DIR}/lib/_ros_setup_buildsh.bash "drcsim"

# don't have rosdep at this point and want gazebo to be cached by docker
DEPENDENCY_PKGS="${DRCSIM_FULL_DEPENDENCIES}"
USE_ROS_REPO=true
OSRF_REPOS_TO_USE="stable"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
