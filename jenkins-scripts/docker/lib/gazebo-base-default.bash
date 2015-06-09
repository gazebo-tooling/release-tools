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

# Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
GAZEBO_MAJOR_VERSION=`\
  grep 'set.*GAZEBO_MAJOR_VERSION ' ${WORKSPACE}/gazebo/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check gazebo version is integer
if ! [[ ${GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GAZEBO_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
DOCKER_JOB_NAME="gazebo_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Step 1: Configure apt
# The image already has all the needed source.lists entries

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
apt-get install -y --force-yes  ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${GAZEBO_EXTRA_DEPENDENCIES} ${EXTRA_PACKAGES}
echo '# END SECTION'

echo '# BEGIN SECTION: install graphic card support'
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
echo '# END SECTION'

# Step 2: configure and build
# Check for DART
if $DART_COMPILE_FROM_SOURCE; then
  echo '# BEGIN SECTION: compiling DART from source'
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
  echo '# END SECTION'
fi

# Normal cmake routine for Gazebo
echo '# BEGIN SECTION: Gazebo configuration'
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build
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
echo '# BEGIN SECTION: UNIT testing'
make test ARGS="-VV -R UNIT_*" || true
echo '# END SECTION'
echo '# BEGIN SECTION: INTEGRATION testing'
make test ARGS="-VV -R INTEGRATION_*" || true
echo '# END SECTION'
echo '# BEGIN SECTION: REGRESSION testing'
make test ARGS="-VV -R REGRESSION_*" || true
echo '# END SECTION'
echo '# BEGIN SECTION: EXAMPLE testing'
make test ARGS="-VV -R EXAMPLE_*" || true
echo '# END SECTION'

# Only run cppcheck on trusty
if [ "$DISTRO" = "trusty" ]; then 
  echo '# BEGIN SECTION: running cppcheck'
  # Step 3: code check
  cd $WORKSPACE/gazebo
  sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
  echo '# END SECTION'
else
  mkdir -p $WORKSPACE/build/cppcheck_results/
  echo "<results></results>" >> $WORKSPACE/build/cppcheck_results/empty.xml 
fi

echo '# BEGIN SECTION: clean build directory and export information'
# Step 4: copy test log
# Broken http://build.osrfoundation.org/job/gazebo-any-devel-precise-amd64-gpu-nvidia/6/console
# Need fix
# mkdir $WORKSPACE/logs
# cp $HOME/.gazebo/logs/*.log $WORKSPACE/logs/

# Step 5. Need to clean build/ directory so disk space is under control
# Move cppcheck and test results out of build
# Copy the results
mv $WORKSPACE/build/cppcheck_results $WORKSPACE/cppcheck_results
mv $WORKSPACE/build/test_results $WORKSPACE/test_results
rm -fr $WORKSPACE/build
mkdir -p $WORKSPACE/build
# To keep backwards compatibility with current configurations keep a copy
# of tests_results in the build path.
cp -a $WORKSPACE/cppcheck_results $WORKSPACE/build/cppcheck_results
cp -a $WORKSPACE/test_results $WORKSPACE/build/test_results
echo '# END SECTION'
DELIM

cat > Dockerfile << DELIM_DOCKER
#######################################################
# Docker file to run build.sh

FROM jrivero/gazebo
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>

# If host is running squid-deb-proxy on port 8000, populate /etc/apt/apt.conf.d/30proxy
# By default, squid-deb-proxy 403s unknown sources, so apt shouldn't proxy ppa.launchpad.net
RUN route -n | awk '/^0.0.0.0/ {print \$2}' > /tmp/host_ip.txt
RUN echo "HEAD /" | nc \$(cat /tmp/host_ip.txt) 8000 | grep squid-deb-proxy \
  && (echo "Acquire::http::Proxy \"http://\$(cat /tmp/host_ip.txt):8000\";" > /etc/apt/apt.conf.d/30proxy) \
  && (echo "Acquire::http::Proxy::ppa.launchpad.net DIRECT;" >> /etc/apt/apt.conf.d/30proxy) \
  || echo "No squid-deb-proxy detected on docker host"


# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
ADD gazebo ${WORKSPACE}/gazebo
RUN echo "${TODAY_STR}"
RUN apt-get update
RUN apt-get install -y ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${GAZEBO_EXTRA_DEPENDENCIES} ${EXTRA_PACKAGES}
ADD build.sh build.sh
RUN chmod +x build.sh
DELIM_DOCKER

sudo rm -fr ${WORKSPACE}/build
mkdir -p ${WORKSPACE}/build

sudo docker pull jrivero/gazebo
sudo docker build -t ${DOCKER_TAG} .
# --priviledged is essential to make DRI work
echo "DISPLAY=unix$DISPLAY"
sudo docker run --privileged \
                       -e "DISPLAY=unix$DISPLAY" \
                       -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                       --cidfile=${CIDFILE} \
                       -t ${DOCKER_TAG} \
                       -v ${WORKSPACE}/build:${WORKSPACE}/build \
                       /bin/bash build.sh

CID=$(cat ${CIDFILE})

sudo docker stop ${CID}
sudo docker rm ${CID}
