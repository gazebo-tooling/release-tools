#!/bin/bash -x

#stop on error
set -e

# Keep the option of default to not really send a build type and let our own gazebo cmake rules
# to decide what is the default mode.
if [ -z ${GZ_BUILD_TYPE} ]; then
    GZ_CMAKE_BUILD_TYPE=
else
    GZ_CMAKE_BUILD_TYPE="-DCMAKE_BUILD_TYPE=${GZ_BUILD_TYPE}"
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
apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${GAZEBO_EXTRA_DEPENDENCIES} ${EXTRA_PACKAGES} git gazebo exuberant-ctags

# Step 2: configure and build

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake ${GZ_CMAKE_BUILD_TYPE} -DENABLE_TESTS_COMPILATION:BOOL=False -DCMAKE_INSTALL_PREFIX=/usr/local $WORKSPACE/gazebo
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
echo \$GAZEBO_LIBS
echo \$GAZEBO_LIBS_LOCAL

mkdir -p $WORKSPACE/abi_checker
cd $WORKSPACE/abi_checker
cat > pkg.xml << CURRENT_DELIM
 <version>
     stable_version
 </version>
 <headers>
   /usr/include/gazebo-2.0/gazebo
 </headers>
 <skip_headers>
   /usr/include/gazebo-2.0/gazebo/GIMPACT
   /usr/include/gazebo-2.0/gazebo/opcode
 </skip_headers>
 <libs>
  \$GAZEBO_LIBS
 </libs>
CURRENT_DELIM

cat > devel.xml << DEVEL_DELIM
 <version>
     development
 </version>
 
  <headers>
   /usr/local/include/gazebo-2.0/gazebo
 </headers>
 
 <skip_headers>
   /usr/local/include/gazebo-2.0/gazebo/GIMPACT
   /usr/local/include/gazebo-2.0/gazebo/opcode
 </skip_headers>
 <libs>
  \$GAZEBO_LIBS_LOCAL
 </libs>
DEVEL_DELIM

abi-compliance-checker -lib gazebo -old pkg.xml -new devel.xml || true
cat /var/lib/jenkins/workspace/gazebo-default-abichecker-precise-amd64/abi_checker/logs/gazebo/stable_version/log.txt
cp compat_reports/gazebo/stable_version_to_development/compat_report.html $WORKSPACE/
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

