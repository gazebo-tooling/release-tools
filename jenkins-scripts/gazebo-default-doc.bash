#!/bin/bash -x

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
GAZEBO_MAJOR_VERSION=`\
  grep 'set.*GAZEBO_MAJOR_VERSION ' ${WORKSPACE}/gazebo/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check gazebo version is integer
if ! [[ ${GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GAZEBO_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: install everything you need

# OSRF repository to get bullet and sdformat
apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Required stuff for Gazebo
apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} 
# Optional stuff for Gazebo
apt-get install -y ${GAZEBO_EXTRA_DEPENDENCIES}
#TODO: pare down to just the following install; blocked on https://bitbucket.org/osrf/gazebo/issue/27/
apt-get install -y doxygen graphviz

# Step 2: configure and build

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/gazebo \
  -DENABLE_TESTS_COMPILATION:Bool=False
make
make doc
./tools/gz sdf -o > dev.html

# Step 3: upload docs
apt-get install -y openssh-client
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@old.gazebosim.org sudo rm -rf /var/www/api/dev /tmp/gazebo_dev /tmp/dev.html
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa -r doxygen/html ubuntu@old.gazebosim.org:/tmp/gazebo_dev
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@old.gazebosim.org sudo mv /tmp/gazebo_dev /var/www/api/dev
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa dev.html ubuntu@old.gazebosim.org:/tmp/dev.html
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@old.gazebosim.org sudo mv /tmp/dev.html /var/www/sdf/dev.html
DELIM

# Copy in my ssh keys, to allow the above ssh/scp calls to work; not sure this is the best way to do it, 
# but it shouldn't be a security issue, as only Jenkins users can see the contents of the workspace
sudo cp $HOME/.ssh/id_rsa $WORKSPACE
sudo chown jenkins $WORKSPACE/id_rsa

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

