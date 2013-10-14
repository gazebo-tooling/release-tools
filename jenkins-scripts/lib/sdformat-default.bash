#!/bin/bash -x
set -e

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: install everything you need ans sdformat (OSRF repo) from binaries
apt-get install -y wget 
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update
apt-get install -y ${BASE_DEPENDENCIES} ${SDFORMAT_BASE_DEPENDENCIES} sdformat git exuberant-ctags 

# Step 2: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/sdformat -DCMAKE_INSTALL_PREFIX=/usr/local
make -j3
make install

# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone git://github.com/lvc/abi-compliance-checker.git  
cd abi-compliance-checker
perl Makefile.pl -install --prefix=/usr

BIN_VERSION=\$(dpkg -l sdformat | tail -n 1 | awk '{ print  \$3 }')

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker
cat > pkg.xml << CURRENT_DELIM
 <version>
     .deb pkg version: \$BIN_VERSION
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
     branch: $BRANCH
 </version>
   
 <headers>
    /usr/local/include/sdformat-1.4/
 </headers>
   
 <libs>
    /usr/local/lib/
 </libs>
DEVEL_DELIM

rm -fr compat_reports/ 
rm -fr $WORKSPACE/compat_report.html
abi-compliance-checker -lib sdformat -old pkg.xml -new devel.xml
# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} $WORKSPACE/ \;
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
