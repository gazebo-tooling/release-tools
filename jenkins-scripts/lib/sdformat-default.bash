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
     stable_version
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
ls
cp compat_reports/sdformat/stable_version_to_development/compat_report.html $WORKSPACE/
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
