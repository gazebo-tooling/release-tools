#!/bin/bash -x
set -e

# Use always GPU in drcsim project
export GPU_SUPPORT_NEEDED=true

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

if [ -z ${GZ_BUILD_TYPE} ]; then
    GZ_CMAKE_BUILD_TYPE=
else
    GZ_CMAKE_BUILD_TYPE="-DCMAKE_BUILD_TYPE=${GZ_BUILD_TYPE}"
fi

# Check if we are using gazebo-any-sdformat-any- script
if [ -z ${GAZEBO_BRANCH} ]; then
    GAZEBO_BRANCH=default
fi
if [ -z ${SDFORMAT_BRANCH} ]; then
    SDFORMAT_BRANCH=default
fi

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get OSRF repo's key
apt-get install -y wget
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Install mercurial and sdformat and gazebo Build-Depends
# but don't install libsdformatN-dev
apt-get install -y mercurial ca-certificates ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES_NO_SDFORMAT} ${GAZEBO_EXTRA_DEPENDENCIES} ${SDFORMAT_BASE_DEPENDENCIES}

# Optional stuff. Check for graphic card support
if ${GRAPHIC_CARD_FOUND}; then
    apt-get install -y ${GRAPHIC_CARD_PKG}
    # Check to be sure version of kernel graphic card support is the same.
    # It will kill DRI otherwise
    CHROOT_GRAPHIC_CARD_PKG_VERSION=\$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print \$3 }' | sed 's:-.*::')
    if [ "\${CHROOT_GRAPHIC_CARD_PKG_VERSION}" != "${GRAPHIC_CARD_PKG_VERSION}" ]; then
       echo "Package ${GRAPHIC_CARD_PKG} has different version in chroot and host system. Maybe you need to update your host"
       exit 1
    fi
fi

# 1. Install sdformat
rm -fr $WORKSPACE/sdformat
hg clone https://bitbucket.org/osrf/sdformat -r $SDFORMAT_BRANCH $WORKSPACE/sdformat

cd $WORKSPACE/sdformat
rm -rf $WORKSPACE/sdformat/build
mkdir -p $WORKSPACE/sdformat/build
cd $WORKSPACE/sdformat/build
cmake -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/sdformat
make -j${MAKE_JOBS}
make install

# 2. Install Gazebo
rm -fr $WORKSPACE/gazebo
hg clone https://bitbucket.org/osrf/gazebo -r $GAZEBO_BRANCH $WORKSPACE/gazebo

rm -rf $WORKSPACE/gazebo/build $WORKSPACE/gazebo/install
mkdir -p $WORKSPACE/gazebo/build $WORKSPACE/gazebo/install
cd $WORKSPACE/gazebo/build
echo '# BEGIN SECTION: Gazebo configuration'
cmake ${GZ_CMAKE_BUILD_TYPE}         \\
    -DCMAKE_INSTALL_PREFIX=/usr      \\
    -DENABLE_SCREEN_TESTS:BOOL=False \\
  $WORKSPACE/gazebo
echo '# END SECTION'
echo '# BEGIN SECTION: Gazebo compilation'
make -j${MAKE_JOBS}
echo '# END SECTION'
echo '# BEGIN SECTION: Gazebo installation'
make install
. /usr/share/gazebo/setup.sh
echo '# END SECTION'

# Need to clean up from previous built
rm -fr $WORKSPACE/cppcheck_results
rm -fr $WORKSPACE/test_results

# Run tests
echo '# BEGIN SECTION: make test'
make test ARGS="-VV" || true
echo '# END SECTION'

echo '# BEGIN SECTION: clean build directory and export information'
# Step 6. Need to clean build/ directory so disk space is under control
# Move test results out of build
# Copy the results
mv $WORKSPACE/gazebo/build/test_results $WORKSPACE/test_results
rm -fr $WORKSPACE/build
mkdir -p $WORKSPACE/build
# To keep backwards compatibility with current configurations keep a copy
# of tests_results in the build path.
cp -a $WORKSPACE/test_results $WORKSPACE/build/test_results
echo '# END SECTION'
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh
