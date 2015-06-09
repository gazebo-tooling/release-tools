#!/bin/bash -x
set -e

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="ros-debian_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

echo '# BEGIN SECTION: set up the PPA'
if [ ${LINUX_DISTRO} == 'ubuntu' ]; then
  apt-add-repository -y ppa:deb-rob/ros-trusty
fi
if [ ${LINUX_DISTRO} = 'debian' ]; then
  echo "deb http://sir.upc.edu/debian-robotics unstable main" \\
						       >> /etc/apt/sources.list
  apt-key adv --keyserver pgp.rediris.es --recv-keys 63DE76AC0B6779BF						       
fi
apt-get update
echo '# END SECTION'

echo '# BEGIN SECTION: install the ros-desktop-full-depends metapackage'
apt-get install -y ros-full-desktop-depends
echo '# END SECTION'

echo '# BEGIN SECTION: compile the rest of desktop-full'
rm -fr ${WORKSPACE}/ros
mkdir ${WORKSPACE}/ros
cd ${WORKSPACE}/ros
rosinstall_generator desktop_full --rosdistro indigo --deps --wet-only --tar --exclude RPP \\
 common_tutorials geometry_tutorials ros_tutorials visualization_tutorials urdf_tutorial \\
 ros_base geometry > indigo-desktop-full-wet.rosinstall

wstool init -j${MAKE_JOBS} src indigo-desktop-full-wet.rosinstall
catkin_make_isolated --install
echo '# END SECTION'

echo '# BEGIN SECTION: run a test'
source ${WORKSPACE}/ros/install_isolated/setup.bash
echo '# END SECTION'
DELIM

# For use ppa
if [[ ${LINUX_DISTRO} == 'ubuntu' ]]; then
  DEPENDENCY_PKGS="python-software-properties apt-utils software-properties-common"
fi

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
