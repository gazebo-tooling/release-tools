#!/bin/bash -x

[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Use always DISPLAY in drcsim project
export GPU_SUPPORT_NEEDED=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: install everything you need

# get ROS repo's key
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Required stuff for drcsim
apt-get install -y ${BASE_DEPENDENCIES} ${DRCSIM_FULL_DEPENDENCIES}
#TODO: pare down to just the following install; blocked on https://bitbucket.org/osrf/gazebo/issue/27/
apt-get install -y doxygen graphviz texlive-latex-base texlive-latex-extra texlive-latex-recommended latex-xcolor texlive-fonts-recommended

# Step 2: configure and build

if [ $DISTRO = quantal ]; then
    rosdep init && rosdep update
fi

# Normal cmake routine
. /opt/ros/${ROS_DISTRO}/setup.sh
. /usr/share/gazebo/setup.sh
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=$WORKSPACE/install $WORKSPACE/drcsim
make doc

# Step 3: upload docs
apt-get install -y openssh-client
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@old.gazebosim.org sudo rm -rf /var/www/drc/api /tmp/drcsim_dev
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa -r doxygen/html ubuntu@old.gazebosim.org:/tmp/drcsim_dev
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@old.gazebosim.org sudo mv /tmp/drcsim_dev /var/www/drc/api
scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa doxygen/latex/drcsim-[0-9]*.[0-9]*.[0-9]*.pdf ubuntu@old.gazebosim.org:/tmp/drcsim-dev.pdf
ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@old.gazebosim.org sudo mv /tmp/drcsim-dev.pdf /var/www/drc/api/drcsim-dev.pdf
DELIM

# Copy in my ssh keys, to allow the above ssh/scp calls to work; not sure this is the best way to do it, 
# but it shouldn't be a security issue, as only Jenkins users can see the contents of the workspace
cp $HOME/.ssh/id_rsa $WORKSPACE

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

