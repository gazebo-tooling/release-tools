#!/bin/bash -x

# Use always GPU in drcsim project unless explicit asked
if [[ -z ${USE_DISPLAY} ]] || ${USE_DISPLAY}; then
  export GPU_SUPPORT_NEEDED=true
fi

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# get ROS repo's key
apt-get install -y wget
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list'
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 1: install everything you need

# Install drcsim's Build-Depends
apt-get install -y --force-yes ${BASE_DEPENDENCIES} ${DRCSIM_FULL_DEPENDENCIES}

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

if [ $DISTRO != precise ]; then
    rosdep init 
    # Hack for not failing when github is down
    update_done=false
    seconds_waiting=0
    while (! \$update_done); do
      rosdep update && update_done=true
      sleep 1
      seconds_waiting=$((seconds_waiting+1))
      [ \$seconds_waiting -gt 60 ] && exit 1
    done
fi

# Normal cmake routine
. /opt/ros/${ROS_DISTRO}/setup.sh
. /usr/share/gazebo/setup.sh
rm -rf $WORKSPACE/build $WORKSPACE/install
mkdir -p $WORKSPACE/build $WORKSPACE/install
cd $WORKSPACE/build

# Do not use atlassiminterface in 32 bist
echo "Check for atlassimitnerface in 32 bits"
if [ "$ARCH" = "i386" ]; then
    EXTRA_ARGS="-DATLAS_SIMINTERFACE_1_BINARY_EXISTS:BOOL=false"
fi

cmake -DCMAKE_INSTALL_PREFIX=$WORKSPACE/install \$EXTRA_ARGS $WORKSPACE/drcsim
make -j${MAKE_JOBS}
make install
SHELL=/bin/sh . $WORKSPACE/install/share/drcsim/setup.sh
export PATH="\$PATH:$WORKSPACE/install/bin/"
#ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV" || true
# Only a selection of tests
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results make test ARGS="-VV -R \\(atlas_publishers_hz_gpu.test\\|atlas_sandia_hands_publishers_hz_gpu.test\\|atlas_rosapi.test\\|atlas_sandia_hands_rosapi.test\\|vrc_task_1_scoring.test\\|vrc_task_1_gzlog_stop.test\\|vrc_task_1_dynamic_walking.test\\|multicamera_connection.test\\|vrc_final_task1_start_standup.test\\|vrc_final_task1_atlas_pubs_gpu.test\\)" || true
ROS_TEST_RESULTS_DIR=$WORKSPACE/build/test_results rosrun rosunit clean_junit_xml.py

# Step 3: code check
if [ "$DISTRO" = "raring" ]; then 
  CODE_CHECK_APP=$WORKSPACE/drcsim/tools/code_check.sh
  if [ -f \$CODE_CHECK_APP ]; then
    cd $WORKSPACE/drcsim
    sh \$CODE_CHECK_APP -xmldir $WORKSPACE/build/cppcheck_results || true
  fi
else
  mkdir -p $WORKSPACE/build/cppcheck_results/
  echo "<results></results>" >> $WORKSPACE/build/cppcheck_results/empty.xml 
fi
DELIM

# Make project-specific changes here
###################################################

sudo pbuilder  --execute \
    --bindmounts $WORKSPACE \
    --basetgz $basetgz \
    -- build.sh

