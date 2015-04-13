#!/bin/bash -x
set -e

# Identify SDFORMAT_MAJOR_VERSION to help with dependency resolution
SDFORMAT_MAJOR_VERSION=`\
  grep 'set.*SDF_MAJOR_VERSION ' ${WORKSPACE}/sdformat/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check sdformat version between 1-9
if ! [[ ${SDFORMAT_MAJOR_VERSION} =~ ^-?[1-9]$ ]]; then
   echo "Error! SDFORMAT_MAJOR_VERSION is not between 1 and 9, check the detection"
   exit -1
fi

echo '# BEGIN SECTION: setup the testing enviroment'
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

echo '# BEGIN SECTION: install dependencies'
# OSRF repository to get ignition-math
apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Step 1: install everything you need
apt-get update
apt-get install -y ${BASE_DEPENDENCIES} ${SDFORMAT_BASE_DEPENDENCIES}
echo '# END SECTION'

# Step 2: configure and build
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
cmake $WORKSPACE/sdformat
make -j${MAKE_JOBS}
make install
make test ARGS="-VV" || true

# Step 3: code check
cd $WORKSPACE/sdformat
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
cat $WORKSPACE/build/cppcheck_results/*.xml
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
