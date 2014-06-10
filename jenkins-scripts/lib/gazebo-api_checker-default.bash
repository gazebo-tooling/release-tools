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

# No explicit activation means no coverage
[ -z ${COVERAGE_ENABLED} ] && COVERAGE_ENABLED=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# Default to install plain gazebo in gazebo_pkg is not speficied
if [[ -z $GAZEBO_PKG ]]; then
    export GAZEBO_PKG=gazebo3
fi

# Split package in gazebo3 needs -dev to explore the headers
if [[ $GAZEBO_PKG == 'gazebo3' ]]; then
    GAZEBO_PKG=libgazebo-dev
fi

# Install gazebo from package and git to retrieve api checker
export EXTRA_PACKAGES="${GAZEBO_PKG} git exuberant-ctags"

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# OSRF repository to get bullet
apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -

# Dart repositories
if $DART_FROM_PKGS; then
  # software-properties for apt-add-repository
  apt-get install -y python-software-properties apt-utils software-properties-common
  apt-add-repository -y ppa:libccd-debs
  apt-add-repository -y ppa:fcl-debs
  apt-add-repository -y ppa:dartsim
fi

if $DART_COMPILE_FROM_SOURCE; then
  apt-get install -y python-software-properties apt-utils software-properties-common git
  apt-add-repository -y ppa:libccd-debs
  apt-add-repository -y ppa:fcl-debs
  apt-add-repository -y ppa:dartsim
fi

# Step 1: install everything you need

# Required stuff for Gazebo
apt-get update
apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${GAZEBO_EXTRA_DEPENDENCIES} ${EXTRA_PACKAGES}

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

# Step 2: configure and build
# Check for DART
if $DART_COMPILE_FROM_SOURCE; then
  if [ -d $WORKSPACE/dart ]; then
      cd $WORKSPACE/dart
      git pull
  else
     git clone https://github.com/dartsim/dart.git $WORKSPACE/dart
  fi
  rm -fr $WORKSPACE/dart/build
  mkdir -p $WORKSPACE/dart/build
  cd $WORKSPACE/dart/build
  cmake .. \
      -DCMAKE_INSTALL_PREFIX=/usr
  #make -j${MAKE_JOBS}
  make -j1
  make install
fi

# Need multiarch to properly compare against the package version
DEB_HOST_MULTIARCH=\$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null)

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake ${GZ_CMAKE_BUILD_TYPE}         \\
    -DCMAKE_INSTALL_PREFIX=/usr/local\\
    -DENABLE_SCREEN_TESTS:BOOL=False \\
    -DCMAKE_INSTALL_LIBDIR:STRING="lib/\${DEB_HOST_MULTIARCH}" \\
  $WORKSPACE/gazebo
make -j${MAKE_JOBS}
make install
. /usr/share/gazebo/setup.sh
make test ARGS="-VV -R UNIT_*" || true
make test ARGS="-VV -R INTEGRATION_*" || true
make test ARGS="-VV -R REGRESSION_*" || true
make test ARGS="-VV -R EXAMPLE_*" || true

# Only run cppcheck on saucy
if [ "$DISTRO" = "saucy" ]; then 
  # Step 3: code check
  cd $WORKSPACE/gazebo
  sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
else
  mkdir -p $WORKSPACE/build/cppcheck_results/
  echo "<results></results>" >> $WORKSPACE/build/cppcheck_results/empty.xml 
fi

# Step 4: copy test log
# Broken http://build.osrfoundation.org/job/gazebo-any-devel-precise-amd64-gpu-nvidia/6/console
# Need fix
# mkdir $WORKSPACE/logs
# cp $HOME/.gazebo/logs/*.log $WORKSPACE/logs/
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

