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


sudo apt-get install -y python openssh-client unzip zip mercurial apache2 redis-server python-pip python-redis
sudo pip install --upgrade boto

sudo pip install unittest-xml-reporting 
sudo apt-get install cmake 
sudo apt-get install python-software-properties


# run redis-server as it does not seem to start on its own
/etc/init.d/redis-server start

rm -rf $WORKSPACE/build $WORKSPACE/cloudsim/test-reports
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/cloudsim
PYTHONPATH=$WORKSPACE/cloudsim/inside/cgi-bin make test ARGS="-VV" || true

DELIM

# Copy in my boto config file, to allow launching of AWS machines.
cp $HOME/boto.test-osrfoundation.org $WORKSPACE/boto.ini

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
