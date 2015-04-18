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

# get ROS repo's key
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list'
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# check for graphic card support
GRAPHIC_TESTS=false
if [ $GRAPHIC_CARD_NAME = Nvidia ] && [ $DISTRO = trusty ]; then
    GRAPHIC_TESTS=true

    if ${GRAPHIC_CARD_FOUND}; then
	apt-get install -y ${GRAPHIC_CARD_PKG}
	# Check to be sure version of kernel graphic card support is the same.
	# It will kill DRI otherwise
	CHROOT_GRAPHIC_CARD_PKG_VERSION=\$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print \$3 }' | sed 's:-.*::')
	if [ "\${CHROOT_GRAPHIC_CARD_PKG_VERSION}" != "${GRAPHIC_CARD_PKG_VERSION}" ]; then
	   echo "Package ${GRAPHIC_CARD_PKG} has different version in chroot and host system. Maybe you need to update your host" 
	   exit 1
	fi
    fi
fi

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
  
  echo "X Error failed is expected on machines with different distro in host than tested"
  echo "We run the test anyway to test ABI or segfaults"
  apt-get install -y psmisc 
  # roslaunch gazebo_ros shapes_world.launch extra_gazebo_args:="--verbose" &
  roslaunch rrbot_gazebo rrbot_world.launch headless:=true extra_gazebo_args:="--verbose" &
  sleep 180
  killall -9 roslaunch || true
  killall -9 gzserver || true 
else
  # timeout --preserve-status 180 roslaunch gazebo_ros shapes_world.launch extra_gazebo_args:="--verbose"
  timeout --preserve-status 180 roslaunch rrbot_gazebo rrbot_world.launch headless:=true extra_gazebo_args:="--verbose"
  if [ $? != 0 ]; then
    echo "Failure response in the launch command" 
    exit 1
  fi
fi

echo "180 testing seconds finished successfully"

DELIM

cat > Dockerfile << DELIM_DOCKER
#######################################################
# Docker file to run build.sh

FROM ubuntu/${DISTRO}
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>

sudo rm -fr ${WORKSPACE}/build
mkdir -p ${WORKSPACE}/build

# Docker file to run build.sh

FROM jrivero/gazebo
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>

# If host is running squid-deb-proxy on port 8000, populate /etc/apt/apt.conf.d/30proxy
# By default, squid-deb-proxy 403s unknown sources, so apt shouldn't proxy ppa.launchpad.net
RUN route -n | awk '/^0.0.0.0/ {print \$2}' > /tmp/host_ip.txt
RUN echo "HEAD /" | nc \$(cat /tmp/host_ip.txt) 8000 | grep squid-deb-proxy \
  && (echo "Acquire::http::Proxy \"http://\$(cat /tmp/host_ip.txt):8000\";" > /etc/apt/apt.conf.d/30proxy) \
  && (echo "Acquire::http::Proxy::ppa.launchpad.net DIRECT;" >> /etc/apt/apt.conf.d/30proxy) \
  || echo "No squid-deb-proxy detected on docker host"


# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
ADD gazebo ${WORKSPACE}/gazebo
RUN echo "${TODAY_STR}"
RUN apt-get install -y wget
# Add repositories to the image
RUN \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list' && \\
    wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - && \\
    sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list' && \\
    wget http://packages.osrfoundation.org/drc.key -O - | apt-key add - && \\
RUN apt-get update
# Check for proper ros wrappers depending on gazebo version
ENV ROS_GAZEBO_PKGS="ros-$ROS_DISTRO-$PACKAGE_ALIAS-msgs \\
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-plugins \\
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros \\
	         ros-$ROS_DISTRO-$PACKAGE_ALIAS-ros-pkgs"
# Need -ros for rosrun
RUN apt-get install -y --force-yes \$ROS_GAZEBO_PKGS ros-$ROS_DISTRO-ros
ADD build.sh build.sh
RUN chmod +x build.sh
DELIM_DOCKER

sudo docker pull ubuntu/${DISTRO}
sudo docker build -t ${DOCKER_TAG} .
# --priviledged is essential to make DRI work
echo "DISPLAY=unix$DISPLAY"
sudo docker run --privileged \
                       -e "DISPLAY=unix$DISPLAY" \
                       -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                       --cidfile=${CIDFILE} \
                       -t ${DOCKER_TAG} \
                       -v ${WORKSPACE}/build:${WORKSPACE}/build \
                       /bin/bash build.sh

CID=$(cat ${CIDFILE})

sudo docker ps 
sudo docker stop ${CID} || true
sudo docker rm ${CID} || true
