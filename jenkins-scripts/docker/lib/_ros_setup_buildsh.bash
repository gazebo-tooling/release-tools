if [ $# -lt 1 ]; then
  echo "Need to call _ros_setup_buildsh.bash with at least one argument"
  exit -1
fi

SOFTWARE_DIR=${1}

if [ -z ${ROS_DISTRO} ]; then
  echo "ROS_DISTRO not set!"
  exit -1
fi

[[ -z ${USE_GZ_VERSION_ROSDEP} ]] && USE_GZ_VERSION_ROSDEP=false
[[ -z ${USE_CATKIN_MAKE} ]] && USE_CATKIN_MAKE=false # use catkin tools by default
[[ -z ${USE_COLCON} ]] && USE_COLCON=false

if ${USE_COLCON}; then
  export CMD_CATKIN_CONFIG=""
  export CMD_CATKIN_LIST="colcon list -g"
  export CMD_CATKIN_BUILD="colcon build --parallel-workers ${MAKE_JOBS} --symlink-install --event-handler console_direct+ ${CATKIN_EXTRA_ARGS}"
  export CMD_CATKIN_TEST="colcon test --parallel-workers 1 --event-handler console_direct+ || true"
  export CMD_CATKIN_TEST_RESULTS="colcon test-result --verbose || true"
  export IGNORE_FILE="COLCON_IGNORE"
elif ${USE_CATKIN_MAKE}; then
  export CMD_CATKIN_CONFIG=""
  export CMD_CATKIN_LIST=""
  export CMD_CATKIN_BUILD="catkin_make -j${MAKE_JOBS} && catkin_make install"
  export CMD_CATKIN_TEST="catkin_make run_tests -j1 || true"
  export CMD_CATKIN_TEST_RESULTS="catkin_test_results || true"
  export IGNORE_FILE="CATKIN_IGNORE"
else
  # catkin tools
  export CMD_CATKIN_CONFIG="catkin config --init --mkdirs"
  export CMD_CATKIN_LIST=""
  export CMD_CATKIN_BUILD="catkin build -j${MAKE_JOBS} --verbose --summary ${CATKIN_EXTRA_ARGS}"
  export CMD_CATKIN_TEST="catkin run_tests -j1 || true"
  export CMD_CATKIN_TEST_RESULTS="catkin_test_results --all --verbose || true"
  export IGNORE_FILE="CATKIN_IGNORE"
fi

export CATKIN_WS="${WORKSPACE}/ws"

cat >> build.sh << DELIM_CONFIG
set -ex

if ${USE_GZ_VERSION_ROSDEP}; then
  apt-get install -y wget
  mkdir -p /etc/ros/rosdep/sources.list.d/
  wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo${GAZEBO_VERSION_FOR_ROS}/00-gazebo${GAZEBO_VERSION_FOR_ROS}.list -O /etc/ros/rosdep/sources.list.d/00-gazebo${GAZEBO_VERSION_FOR_ROS}.list
fi

if [ `expr length "${ROS_SETUP_PREINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre install hook'
${ROS_SETUP_PREINSTALL_HOOK}
echo '# END SECTION'
fi

echo '# BEGIN SECTION: run rosdep'
# Step 2: configure and build
[[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]] && rosdep init
# Hack for not failing when github is down
update_done=false
seconds_waiting=0
while (! \$update_done); do
  rosdep update && update_done=true
  sleep 1
  seconds_waiting=$((seconds_waiting+1))
  [ \$seconds_waiting -gt 60 ] && exit 1
done

SHELL=/bin/sh . /opt/ros/${ROS_DISTRO}/setup.sh
mkdir -p \$HOME/.gazebo
echo '# END SECTION'

echo '# BEGIN SECTION: create the catkin workspace'
rm -fr ${CATKIN_WS}
mkdir -p ${CATKIN_WS}/src
cd ${CATKIN_WS}
${CMD_CATKIN_CONFIG}
ln -s "${WORKSPACE}/${SOFTWARE_DIR}" "${CATKIN_WS}/src/${SOFTWARE_DIR}"
${CMD_CATKIN_LIST}
echo '# END SECTION'
DELIM_CONFIG

if [ `expr length "${ROS_WS_PREBUILD_HOOK} "` -gt 1 ]; then
cat >> build.sh << DELIM_PREBUILD_HOOK
cd ${CATKIN_WS}/src
${ROS_WS_PREBUILD_HOOK}
cd ${CATKIN_WS}
DELIM_PREBUILD_HOOK
fi

cat >> build.sh << DELIM_COMPILATION
echo '# END SECTION'

echo '# BEGIN SECTION install the system dependencies'
${CMD_CATKIN_LIST}
rosdep install --from-paths . \
               -r             \
               --ignore-src   \
               --rosdistro=${ROS_DISTRO} \
               --default-yes \
               --as-root apt:false
# Package installation could bring some local_setup.sh files with it
# need to source it again after installation
SHELL=/bin/sh . /opt/ros/${ROS_DISTRO}/setup.sh
echo '# END SECTION'

echo '# BEGIN SECTION compile the catkin workspace'
${CMD_CATKIN_BUILD}
echo '# END SECTION'

echo '# BEGIN SECTION: running tests'
# some tests needs to source install before running
source install/setup.bash || true
# need to ignore build and install directories per
# https://github.com/ament/ament_lint/issues/48#issuecomment-320129800
[[ -d install/ ]] && touch install/${IGNORE_FILE}
[[ -d build/ ]] && touch build/${IGNORE_FILE}
${CMD_CATKIN_TEST}
${CMD_CATKIN_TEST_RESULTS}

# link test results to usual place
mkdir -p ${WORKSPACE}/build/test_results
DIRS=\$(find . -name test_results -type d)
for d in \$DIRS; do
  for t in \$(find \$d -name "*.xml" -type f) ; do
     mv \$t ${WORKSPACE}/build/test_results/\$(basename \$(dirname \$t))_\$(basename \$t)
  done
done
echo '# END SECTION'

if [ `expr length "${ROS_SETUP_POSTINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running post install hook'
${ROS_SETUP_POSTINSTALL_HOOK}
echo '# END SECTION'
fi
DELIM_COMPILATION
