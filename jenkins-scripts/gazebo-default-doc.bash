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
rm -f $WORKSPACE/pbuilder
wget -O $WORKSPACE/pbuilder http://bazaar.launchpad.net/~vcs-imports/pbuilder/trunk/download/head:/pbuilder/pbuilder
chmod a+x $WORKSPACE/pbuilder

# Setup the pbuilder environment if not existing, or update
if [ ! -e $basetgz ] || [ ! -s $basetgz ] 
then
  #make sure the base dir exists
  sudo mkdir -p $base
  #create the base image
  sudo $WORKSPACE/pbuilder create \
    --distribution $distro \
    --aptconfdir $rootdir/etc/apt \
    --basetgz $basetgz \
    --architecture $arch
else
  sudo $WORKSPACE/pbuilder --update --basetgz $basetgz
fi

# Boilerplate.
# DO NOT MODIFY
###################################################

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: install everything you need

# Required stuff for Gazebo
apt-get install -y build-essential libtinyxml-dev libtbb-dev libxml2-dev libqt4-dev pkg-config  libprotoc-dev libfreeimage-dev libprotobuf-dev protobuf-compiler libboost-all-dev freeglut3-dev cmake libogre-dev libcurl4-openssl-dev libtar-dev
# Optional stuff for Gazebo
apt-get install -y robot-player-dev libcegui-mk2-dev libavformat-dev libavcodec-dev libltdl-dev libgtest-dev
#TODO: pare down to just the following install; blocked on https://bitbucket.org/osrf/gazebo/issue/27/
apt-get install -y doxygen graphviz texlive-latex-base texlive-latex-extra texlive-latex-recommended latex-xcolor texlive-fonts-recommended

# Step 2: configure and build

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/gazebo
make doc
make gzsdf
GAZEBO_RESOURCE_PATH=`pwd`/../gazebo/gazebo ./tools/gzsdf doc > dev.html

# Step 3: upload docs
apt-get install -y openssh-client
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@gazebosim.org sudo rm -rf /var/www/api/dev /tmp/gazebo_dev /tmp/dev.html
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa -r doxygen/html ubuntu@gazebosim.org:/tmp/gazebo_dev
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@gazebosim.org sudo mv /tmp/gazebo_dev /var/www/api/dev
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa doxygen/latex/gazebo-[0-9]*.[0-9]*.[0-9]*.pdf ubuntu@gazebosim.org:/tmp/gazebo-dev.pdf
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@gazebosim.org sudo mv /tmp/gazebo-dev.pdf /var/www/api/gazebo-dev.pdf
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa dev.html ubuntu@gazebosim.org:/tmp/dev.html
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@gazebosim.org sudo mv /tmp/dev.html /var/www/sdf/dev.html
DELIM

# Copy in my ssh keys, to allow the above ssh/scp calls to work; not sure this is the best way to do it, 
# but it shouldn't be a security issue, as only Jenkins users can see the contents of the workspace
cp $HOME/.ssh/id_rsa $WORKSPACE

# Make project-specific changes here
###################################################

sudo $WORKSPACE/pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

