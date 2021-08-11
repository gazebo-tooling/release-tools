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

export CATKIN_WS="${WORKSPACE}/ws"

cat >> build.sh << DELIM_CONFIG
set -ex

if ${USE_GZ_VERSION_ROSDEP}; then
  sudo apt-get install -y wget
  sudo mkdir -p /etc/ros/rosdep/sources.list.d/
  sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo${GAZEBO_VERSION_FOR_ROS}/00-gazebo${GAZEBO_VERSION_FOR_ROS}.list -O /etc/ros/rosdep/sources.list.d/00-gazebo${GAZEBO_VERSION_FOR_ROS}.list
fi

if [ `expr length "${ROS_SETUP_PREINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre install hook'
${ROS_SETUP_PREINSTALL_HOOK}
echo '# END SECTION'
fi

echo '# BEGIN SECTION: run rosdep'
# Step 2: configure and build
[[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]] && sudo rosdep init
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
echo '# END SECTION'

echo '# BEGIN SECTION: create the catkin workspace'
rm -fr ${CATKIN_WS}
mkdir -p ${CATKIN_WS}/src
cd ${CATKIN_WS}
ln -s "${WORKSPACE}/${SOFTWARE_DIR}" "${CATKIN_WS}/src/${SOFTWARE_DIR}"
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
rosdep install --from-paths . \
               --ignore-src   \
               --rosdistro=${ROS_DISTRO} \
               --default-yes
echo '# END SECTION'

echo '# BEGIN SECTION compile the catkin workspace'
catkin_make_isolated -j${MAKE_JOBS}
echo '# END SECTION'

echo '# BEGIN SECTION: running tests'
catkin_make_isolated --catkin-make-args run_tests -j1 || true
catkin_test_results --all --verbose || true

# link test results to usual place
mkdir -p ${WORKSPACE}/build/test_results
DIRS=\$(find . -name test_results -type d)
for d in \$DIRS; do
  for t in \$(find \$d -name "*.xml" -type f) ; do
     mv \$t ${WORKSPACE}/build/test_results
 done
done
echo '# END SECTION'

if [ `expr length "${ROS_SETUP_POSTINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre TEST hook'
${ROS_SETUP_POSTINSTALL_HOOK}
echo '# END SECTION'
fi
DELIM_COMPILATION
