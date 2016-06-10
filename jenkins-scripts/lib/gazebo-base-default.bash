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

[ -z ${COVERAGE_ENABLED} ] && COVERAGE_ENABLED=false

# Identify GAZEBO_MAJOR_VERSION to help with dependency resolution
GAZEBO_MAJOR_VERSION=`\
  grep 'set.*GAZEBO_MAJOR_VERSION ' ${WORKSPACE}/gazebo/CMakeLists.txt | \
  tr -d 'a-zA-Z _()'`

# Check gazebo version is integer
if ! [[ ${GAZEBO_MAJOR_VERSION} =~ ^-?[0-9]+$ ]]; then
  echo "Error! GAZEBO_MAJOR_VERSION is not an integer, check the detection"
  exit -1
fi

NEED_PRERELEASE=false
if [[ $GAZEBO_MAJOR_VERSION -ge 7 ]]; then
  # Need prerelease repo to get sdformat4 during the development cycle
  # 20160125 release date of gazebo7
  if [[ $(date +%Y%m%d) -le 20160125 ]]; then
    NEED_PRERELEASE=true
  fi
fi

echo '# BEGIN SECTION: setup the testing enviroment'
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

if ${COVERAGE_ENABLED} ; then
  # Workaround on problem with setting HOME to /var/lib/jenkins
  if [[ -f $HOME/bullseye-jenkins-license ]]; then
      LICENSE_FILE="$HOME/bullseye-jenkins-license"
  else
      LICENSE_FILE="/var/lib/jenkins/bullseye-jenkins-license"
  fi

  set +x # keep password secret
  BULLSEYE_LICENSE=`cat $LICENSE_FILE`
  set -x # back to debug
fi
echo '# END SECTION'

cat > build.sh << DELIM
#!/bin/bash
###################################################
# Make project-specific changes here
#
set -ex
source ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}

echo '# BEGIN SECTION: install dependencies'
init_stopwatch INSTALL_DEPENDENCIES
# OSRF repository to get bullet
apt-get install -y wget
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
if ${NEED_PRERELEASE}; then
  sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu-prerelease ${DISTRO} main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
fi
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


if ${COVERAGE_ENABLED} ; then
  # Clean previous content
  rm -fr $WORKSPACE/coverage
  # Download and install Bullseyes
  cd $WORKSPACE
  rm -fr $WORKSPACE/Bulls*
  
  # Look for current version. NOT IN USE since we lost the maintenance support on 2014 
  # reenable if the support is back.
  # wget http://www.bullseye.com/download/ -O bull_index.html
  # BULL_TAR=\$( grep -R BullseyeCoverage-.*-Linux-x64.tar bull_index.html | head -n 1 | sed 's/.*">//' | sed 's/<.*//' )
  # wget http://www.bullseye.com/download/\$BULL_TAR -O bullseye.tar

  # Download package
  wget https://www.dropbox.com/s/i1ay7t8sg8i77jr/bullseye-8.8.9.tar -O bullseye.tar
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

stop_stopwatch INSTALL_DEPENDENCIES
stop_stopwatch CREATE_TESTING_ENVIROMENT

# Normal cmake routine for Gazebo
echo '# BEGIN SECTION: Gazebo configuration'
init_stopwatch COMPILATION
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
stop_stopwatch COMPILATION
echo '# END SECTION'

# Need to clean up from previous built
rm -fr $WORKSPACE/cppcheck_results
rm -fr $WORKSPACE/test_results

# Run tests
echo '# BEGIN SECTION: UNIT testing'
init_stopwatch UNIT_TESTING
make test ARGS="-VV -R UNIT_*" || true
stop_stopwatch UNIT_TESTING
echo '# END SECTION'
echo '# BEGIN SECTION: INTEGRATION testing'
init_stopwatch INTEGRATION_TESTING
make test ARGS="-VV -R INTEGRATION_*" || true
stop_stopwatch INTEGRATION_TESTING
echo '# END SECTION'
echo '# BEGIN SECTION: REGRESSION testing'
init_stopwatch REGRESSION_TESTING
make test ARGS="-VV -R REGRESSION_*" || true
stop_stopwatch REGRESSION_TESTING
echo '# END SECTION'
echo '# BEGIN SECTION: EXAMPLE testing'
init_stopwatch EXAMPLE_TESTING
make test ARGS="-VV -R EXAMPLE_*" || true
stop_stopwatch EXAMPLE_TESTING
echo '# END SECTION'

echo '# BEGIN SECTION: running cppcheck'
init_stopwatch CPPCHECK
# Step 3: code check
cd $WORKSPACE/gazebo
sh tools/code_check.sh -xmldir $WORKSPACE/build/cppcheck_results || true
stop_stopwatch CPPCHECK
echo '# END SECTION'

# Step 4: generate code coverage if enabled
if ${COVERAGE_ENABLED} ; then
  rm -fr $WORKSPACE/coverage
  rm -fr $WORKSPACE/bullshtml
  mkdir -p $WORKSPACE/coverage
  covselect --add '!$WORKSPACE/build/' '!../build/' '!test/' '!tools/test/' '!deps/' '!/opt/' '!gazebo/rendering/skyx/' '!/tmp/'
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

echo '# BEGIN SECTION: clean build directory and export information'
# Step 5: copy test log
# Broken http://build.osrfoundation.org/job/gazebo-any-devel-precise-amd64-gpu-nvidia/6/console
# Need fix
# mkdir $WORKSPACE/logs
# cp $HOME/.gazebo/logs/*.log $WORKSPACE/logs/

# Step 6. Need to clean build/ directory so disk space is under control
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

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

stop_stopwatch TOTAL_TIME
