#!/bin/bash -x

###################################################
# Boilerplate.
# DO NOT MODIFY

#stop on error
set -e

distro=precise
arch=amd64
base=/var/cache/pbuilder-$distro-$arch

aptconffile=$WORKSPACE/apt.conf

#increment this value if you have changed something that will invalidate base tarballs. #TODO this will need cleanup eventually.
basetgz_version=2

rootdir=$base/apt-conf-$basetgz_version

basetgz=$base/base-$basetgz_version.tgz
output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

sudo apt-get update
sudo apt-get install -y pbuilder python-empy python-argparse debhelper # todo move to server setup, or confirm it's there

if [ -e $WORKSPACE/catkin-debs ]
then
  rm -rf $WORKSPACE/catkin-debs
fi

git clone git://github.com/willowgarage/catkin-debs.git $WORKSPACE/catkin-debs -b master --depth 1


cd $WORKSPACE/catkin-debs
. setup.sh

#setup the cross platform apt environment
# using sudo since this is shared with pbuilder and if pbuilder is interupted it will leave a sudo only lock file.  Otherwise sudo is not necessary. 
# And you can't chown it even with sudo and recursive 
sudo PYTHONPATH=$PYTHONPATH $WORKSPACE/catkin-debs/scripts/setup_apt_root.py $distro $arch $rootdir --local-conf-dir $WORKSPACE --repo ros@http://packages.ros.org/ros/ubuntu

sudo rm -rf $output_dir
mkdir -p $output_dir

sudo rm -rf $work_dir
mkdir -p $work_dir
cd $work_dir

sudo apt-get update -c $aptconffile

# Grab a newer version of pbuilder, because the one that ships with Lucid suffers from a bug when using --execute
# https://bugs.launchpad.net/ubuntu/+source/pbuilder/+bug/811016
rm -f pbuilder
wget -O pbuilder http://bazaar.launchpad.net/~vcs-imports/pbuilder/trunk/download/head:/pbuilder/pbuilder
chmod a+x pbuilder

# Setup the pbuilder environment if not existing, or update
if [ ! -e $basetgz ] || [ ! -s $basetgz ] 
then
  #make sure the base dir exists
  sudo mkdir -p $base
  #create the base image
  sudo pbuilder create \
    --distribution $distro \
    --aptconfdir $rootdir/etc/apt \
    --basetgz $basetgz \
    --architecture $arch
else
  sudo pbuilder --update --basetgz $basetgz
fi

# Boilerplate.
# DO NOT MODIFY
###################################################

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get ROS repo's key, to be used both in installing prereqs here and in creating the pbuilder chroot
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Required stuff for Gazebo
apt-get install -y cmake build-essential debhelper libfreeimage-dev libprotoc-dev libprotobuf-dev protobuf-compiler freeglut3-dev libcurl4-openssl-dev libtinyxml-dev libtar-dev libtbb-dev libogre-dev libxml2-dev pkg-config libqt4-dev ros-fuerte-urdfdom ros-fuerte-console-bridge libltdl-dev libboost-thread-dev libboost-signals-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev cppcheck robot-player-dev libcegui-mk2-dev libavformat-dev libavcodec-dev libswscale-dev

# Step 2: configure and build

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
CMAKE_PREFIX_PATH=/opt/ros/fuerte cmake -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/gazebo
make -j1
make install
. /usr/share/gazebo/setup.sh
LD_LIBRARY_PATH=/opt/ros/fuerte/lib make test ARGS="-VV" || true

# Step 3: code check
cd $WORKSPACE/gazebo
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
