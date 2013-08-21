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

set +x # keep password secret
BULLSEYE_LICENSE=`cat $HOME/bullseye-jenkins-license`
set -x # back to debug

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

# Required stuff for Gazebo
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


if ${COVERAGE_ENABLED} ; then
  # Clean previous content
  rm -fr $WORKSPACE/coverage
  # Download and install Bullseyes
  cd $WORKSPACE
  rm -fr $WORKSPACE/Bulls*
  # Look for current version
  wget http://www.bullseye.com/download/ -O bull_index.html
  BULL_TAR=\$( grep -R BullseyeCoverage-.*-Linux-x64.tar bull_index.html | head -n 1 | sed 's/.*">//' | sed 's/<.*//' )
  # Download package
  wget http://www.bullseye.com/download/\$BULL_TAR -O bullseye.tar
  tar -xf bullseye.tar
  cd Bulls*
  # Set up the license
  echo $PATH >install-path
  rm -fr /usr/bullseyes
  set +x # keep password secret
  ./install --prefix /usr/bullseyes --key $BULLSEYE_LICENSE
  set -x # back to debug
  # Set up Bullseyes for compiling
  export PATH=/usr/bullseyes/bin:\$PATH
  export COVFILE=$WORKSPACE/gazebo/test.cov
  cd $WORKSPACE/gazebo
  covselect --file test.cov --add .
  cov01 --on
fi

# Step 2: configure and build

# Normal cmake routine for Gazebo
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
cmake ${GZ_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=/usr $WORKSPACE/gazebo
make -j${MAKE_JOBS}
make install
. /usr/share/gazebo/setup.sh
make test ARGS="-VV" || true

# Step 3: code check
cd $WORKSPACE/gazebo
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true

# Step 4: generate code coverage if enabled
if ${COVERAGE_ENABLED} ; then
  rm -fr $WORKSPACE/coverage
  rm -fr $WORKSPACE/bullshtml
  mkdir -p $WORKSPACE/coverage
  covselect --add '!$WORKSPACE/build/' '!../build/' '!test/' '!tools/test/' '!deps/' '!/opt/' '!gazebo/rendering/skyx/'
  covhtml --srcdir $WORKSPACE/gazebo/ $WORKSPACE/coverage
  # Generate valid cover.xml file using the bullshtml software
  # java is needed to run bullshtml
  apt-get install -y default-jre
  cd $WORKSPACE
  wget http://bullshtml.googlecode.com/files/bullshtml_1.0.5.tar.gz -O bullshtml.tar.gz
  tar -xzf bullshtml.tar.gz
  cd bullshtml
  sh bullshtml .
  # Hack to remove long paths from report
  find . -name '*.html' -exec sed -i -e 's:${WORKSPACE}::g' {} \;
fi
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

