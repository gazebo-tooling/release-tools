#!/bin/bash -x

#stop on error
set -e


. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# OSRF repository to get bullet
apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Required stuff for Gazebo and install gazebo binary itself
apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${GAZEBO_EXTRA_DEPENDENCIES} ${EXTRA_PACKAGES} mercurial git exuberant-ctags

# Step 2: configure and build

# 2.1 Origin branch
cd $WORKSPACE/gazebo
hg up $GAZEBO_ORIGIN_BRANCH
# Normal cmake routine for Gazebo
cd $WORKSPACE/build
cmake -DENABLE_TESTS_COMPILATION:BOOL=False \\
      -DCMAKE_INSTALL_PREFIX=/usr/local/origin_branch \\
  $WORKSPACE/gazebo
make -j${MAKE_JOBS}
make install

# 2.2 Target branch
# Reusing the same building and source directory to save bandwith and
# compilation time.
cd $WORKSPACE/gazebo
hg up $GAZEBO_TARGET_BRANCH
# Normal cmake routine for Gazebo
cd $WORKSPACE/build
cmake -DENABLE_TESTS_COMPILATION:BOOL=False \\
      -DCMAKE_INSTALL_PREFIX=/usr/local/target_branch \\
  $WORKSPACE/gazebo
make -j${MAKE_JOBS}
make install

# Install abi-compliance-checker.git
cd $WORKSPACE
rm -fr $WORKSPACE/abi-compliance-checker
git clone git://github.com/lvc/abi-compliance-checker.git  
cd abi-compliance-checker
perl Makefile.pl -install --prefix=/usr

GAZEBO_LIBS=\$(dpkg -L gazebo | grep lib.*.so)
GAZEBO_LIBS_LOCAL=\$(dpkg -L gazebo | grep lib.*.so | sed -e 's:^/usr:/usr/local:g')

BIN_VERSION=\$(dpkg -l gazebo | tail -n 1 | awk '{ print  \$3 }')

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker
cat > pkg.xml << CURRENT_DELIM
 <version>
     branch: $GAZEBO_ORIGIN_BRANCH
 </version>

 <headers>
   /usr/local/origin_branch/include/gazebo-2.0/gazebo
 </headers>

 <skip_headers>
   /usr/local/origin_branch/include/gazebo-2.0/gazebo/GIMPACT
   /usr/local/origin_branch/include/gazebo-2.0/gazebo/opcode
   /usr/local/origin_branch/include/gazebo-2.0/gazebo/test
 </skip_headers>

 <libs>
   /usr/local/origin_branch/lib/
 </libs>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $GAZEBO_TARGET_BRANCH
 </version>
 
 <headers>
   /usr/local/target_branch/include/gazebo-2.0/gazebo
 </headers>

 <skip_headers>
   /usr/local/target_branch/include/gazebo-2.0/gazebo/GIMPACT
   /usr/local/target_branch/include/gazebo-2.0/gazebo/opcode
   /usr/local/target_branch/include/gazebo-2.0/gazebo/test
 </skip_headers>

 <libs>
   /usr/local/target_branch/lib/
 </libs>
DEVEL_DELIM

# clean previous reports
rm -fr $WORKSPACE/compat_report.html
rm -fr compat_reports/
# run report tool
abi-compliance-checker -lib gazebo -old pkg.xml -new devel.xml || true
# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} $WORKSPACE/ \;
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

