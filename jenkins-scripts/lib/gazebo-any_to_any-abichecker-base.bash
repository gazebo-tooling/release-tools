#!/bin/bash -x

#stop on error
set -e

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

# No GAZEBO_PKG specified checking latest
if [[ -z ${GAZEBO_PKG} ]]; then 
  # Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
  GAZEBO_MAJOR_VERSION=`\
    grep 'set.*GAZEBO_MAJOR_VERSION ' ${WORKSPACE}/gazebo/CMakeLists.txt | \
    tr -d 'a-zA-Z _()'`

  # Check gazebo version is integer
  if ! [[ ${GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
    echo "Error! GAZEBO_MAJOR_VERSION is not an integer, check the detection"
    exit -1
  fi

  GAZEBO_LATEST_RELEASE=$((GAZEBO_MAJOR_VERSION - 1))
  GAZEBO_PKG=libgazebo${GAZEBO_LATEST_RELEASE}-dev
fi

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
apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${GAZEBO_EXTRA_DEPENDENCIES} ${EXTRA_PACKAGES} mercurial ca-certificates git exuberant-ctags

# Step 2: configure and build

# Copy gazebo to manipulate under root perms
# 2.1 Origin branch
cp -a $WORKSPACE/gazebo /tmp/gazebo
chown -R root:root /tmp/gazebo
cd /tmp/gazebo
hg pull --rev ${GAZEBO_ORIGIN_BRANCH}
hg up $GAZEBO_ORIGIN_BRANCH
# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake -DENABLE_TESTS_COMPILATION:BOOL=False \\
      -DCMAKE_INSTALL_PREFIX=/usr/local/origin_branch \\
  /tmp/gazebo
make -j${MAKE_JOBS}
make install
GAZEBO_ORIGIN_DIR=\$(find /usr/local/origin_branch/include -name gazebo-* -type d | sed -e 's:.*/::')

# 2.2 Target branch
# Reusing the same building and source directory to save bandwith and
# compilation time.
cd /tmp/gazebo
hg pull --rev ${GAZEBO_TARGET_BRANCH}
hg up $GAZEBO_TARGET_BRANCH
# Normal cmake routine for Gazebo
cd $WORKSPACE/build
cmake -DENABLE_TESTS_COMPILATION:BOOL=False \\
      -DCMAKE_INSTALL_PREFIX=/usr/local/target_branch \\
  /tmp/gazebo
make -j${MAKE_JOBS}
make install
GAZEBO_TARGET_DIR=\$(find /usr/local/target_branch/include -name gazebo-* -type d | sed -e 's:.*/::')

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
     branch: $GAZEBO_ORIGIN_BRANCH
 </version>

 <headers>
   /usr/local/origin_branch/include/\$GAZEBO_ORIGIN_DIR/gazebo
 </headers>

 <skip_headers>
   /usr/local/origin_branch/include/\$GAZEBO_ORIGIN_DIR/gazebo/GIMPACT
   /usr/local/origin_branch/include/\$GAZEBO_ORIGIN_DIR/gazebo/opcode
   /usr/local/origin_branch/include/\$GAZEBO_ORIGIN_DIR/gazebo/test
 </skip_headers>

 <libs>
   /usr/local/origin_branch/lib/
 </libs>

 <gcc_options>
     -std=c++11
 </gcc_options>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     branch: $GAZEBO_TARGET_BRANCH
 </version>
 
 <headers>
   /usr/local/target_branch/include/\$GAZEBO_TARGET_DIR/gazebo
 </headers>

 <skip_headers>
   /usr/local/target_branch/include/\$GAZEBO_TARGET_DIR/gazebo/GIMPACT
   /usr/local/target_branch/include/\$GAZEBO_TARGET_DIR/gazebo/opcode
   /usr/local/target_branch/include/\$GAZEBO_TARGET_DIR/gazebo/test
 </skip_headers>

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
abi-compliance-checker -lib gazebo -old pkg.xml -new devel.xml || true
# copy method version independant ( cp ... /*/ ... was not working)
find compat_reports/ -name compat_report.html -exec cp {} $WORKSPACE/ \;

# Clean up disk space
rm -rf $WORKSPACE/build
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

