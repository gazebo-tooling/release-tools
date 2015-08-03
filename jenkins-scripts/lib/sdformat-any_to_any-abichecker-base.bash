#!/bin/bash -x

#stop on error
set -e

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

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
apt-get install -y ${BASE_DEPENDENCIES} ${SDFORMAT_BASE_DEPENDENCIES} git exuberant-ctags mercurial ca-certificates

# Step 2: configure and build

# Copy sdformat to manipulate under root perms
# 2.1 Origin branch
cp -a $WORKSPACE/sdformat /tmp/sdformat
chown -R root:root /tmp/sdformat
cd /tmp/sdformat
hg pull
hg up $SDFORMAT_ORIGIN_BRANCH
# Normal cmake routine for sdformat
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/origin_branch \\
  /tmp/sdformat
make -j${MAKE_JOBS}
make install
sdformat_ORIGIN_DIR=\$(find /usr/local/origin_branch/include -name sdformat-* -type d | sed -e 's:.*/::')

# 2.2 Target branch
# Reusing the same building and source directory to save bandwith and
# compilation time.
cd /tmp/sdformat
hg up $SDFORMAT_TARGET_BRANCH
# Normal cmake routine for sdformat
cd $WORKSPACE/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/target_branch \\
  /tmp/sdformat
make -j${MAKE_JOBS}
make install
sdformat_TARGET_DIR=\$(find /usr/local/target_branch/include -name sdformat-* -type d | sed -e 's:.*/::')

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
     branch: $SDFORMAT_ORIGIN_BRANCH
 </version>

 <headers>
   /usr/local/origin_branch/include/\$sdformat_ORIGIN_DIR/sdf
 </headers>

 <libs>
   /usr/local/origin_branch/lib/
 </libs>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $SDFORMAT_TARGET_BRANCH
 </version>
 
 <headers>
   /usr/local/target_branch/include/\$sdformat_TARGET_DIR/sdf
 </headers>

 <libs>
   /usr/local/target_branch/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>
DEVEL_DELIM

# clean previous reports
rm -fr $WORKSPACE/compat_report.html
rm -fr compat_reports/
# run report tool
abi-compliance-checker -lib sdformat -old pkg.xml -new devel.xml || true
# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} $WORKSPACE/ \;
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

