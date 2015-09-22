#!/bin/bash -x

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

DOCKER_JOB_NAME="ros_gazebo_pkgs_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 2: configure and build
rosdep init 
# Hack for not failing when github is down
update_done=false
seconds_waiting=0
while (! \$update_done); do
  rosdep update && update_done=true
  sleep 1
  seconds_waiting=$((seconds_waiting+1))
  [ \$seconds_waiting -gt 60 ] && exit 1
done

SHELL=/bin/sh . /opt/ros/${ROS_DISTRO}/setup.sh

# In our nvidia machines, run the test to launch altas
# Seems like there is no failure in runs on precise pbuilder in
# our trusty machine. So we do not check for GRAPHIC_TESTS=true
mkdir -p \$HOME/.gazebo

# Create the catkin workspace
rm -fr $WORKSPACE/ws/src
mkdir -p $WORKSPACE/ws/src
cd $WORKSPACE/ws/src
catkin_init_workspace
git clone https://github.com/ros-simulation/gazebo_ros_demos
cd gazebo_ros_demos/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD
cd $WORKSPACE/ws
catkin_make -j${MAKE_JOBS}
SHELL=/bin/sh . $WORKSPACE/ws/devel/setup.sh

# Precise coreutils does not support preserve-status
if [ $DISTRO = 'precise' ]; then
  # TODO: all the testing on precise X in our jenkins is currently segfaulting
  # docker can not run on Precise without altering the base packages
  # Previous attemps to get GPU acceleration for intel in chroot:
  # http://build.osrfoundation.org/job/ros_gazebo_pkgs-release-testing-broken-intel/
  roslaunch gazebo_ros shapes_world.launch extra_gazebo_args:="--verbose" &

  sleep 180
  # Install psmisc for killall while running
  apt-get install -y psmisc 
  killall -9 roslaunch || true
  killall -9 gzserver || true 
else
  timeout --preserve-status 180 roslaunch rrbot_gazebo rrbot_world.launch headless:=true extra_gazebo_args:="--verbose"
  if [ $? != 0 ]; then
    echo "Failure response in the launch command" 
    exit 1
  fi
fi

echo "180 testing seconds finished successfully"
DELIM

USE_OSRF_REPO=true
USE_ROS_REPO=true

# Let's try to install all the packages and check the example
ROS_GAZEBO_PKGS="ros-$ROS_DISTRO-$PACKAGE_ALIAS-msgs \
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-plugins \
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros \
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros-pkgs"

DEPENDENCY_PKGS="${ROS_GAZEBO_PKGS} ${ROS_GAZEBO_PKGS_EXAMPLE_DEPS} git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
