#!/bin/bash -x
set -e

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need
apt-get install -y python cmake build-essential libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev libtinyxml-dev cppcheck sdformat exuberant-ctags git 

# Step 2: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/sdformat -DCMAKE_INSTALL_PREFIX=/usr/local
make -j3
make install
make test ARGS="-VV" || true

# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone git://github.com/lvc/abi-compliance-checker.git  
cd abi-compliance-checker
perl Makefile.pl -install --prefix=/usr

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker
cat > pkg.xml << CURRENT_DELIM
 <version>
     stable version
 </version>
   
 <headers>
    /usr/include/sdformat-1.4/
 </headers>
   
 <libs>
    /usr/lib/
 </libs>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     development
 </version>
   
 <headers>
    /usr/local/include/sdformat-1.4//
 </headers>
   
 <libs>
    /usr/local/lib/
 </libs>
DEVEL_DELIM

abi-compliance-checker -lib sdformat -old pkg.xml -new devel.xml
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
