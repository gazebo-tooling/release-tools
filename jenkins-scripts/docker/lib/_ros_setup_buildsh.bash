if [ $# -lt 1 ]; then
  echo "Need to call _ros_setup_buildsh.bash with at least one argument"
  exit -1
fi

SOFTWARE_DIR=${1}

if [ -z ${ROS_DISTRO} ]; then
  echo "ROS_DISTRO not set!"
  exit -1
fi

export CATKIN_WS="${WORKSPACE}/ws"

cat > build.sh << DELIM_CONFIG
set -ex

echo '# BEGIN SECTION: run rosdep'
# Step 2: configure and build
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

SHELL=/bin/sh . /opt/ros/${ROS_DISTRO}/setup.sh

# In our nvidia machines, run the test to launch altas
# Seems like there is no failure in runs on precise pbuilder in
# our trusty machine. So we do not check for GRAPHIC_TESTS=true
mkdir -p \$HOME/.gazebo
echo '# END SECTION'

echo '# BEGIN SECTION: create the catkin workspace'
rm -fr ${CATKIN_WS}
mkdir -p ${CATKIN_WS}/src
cd ${CATKIN_WS}
catkin config --init --mkdirs
ln -s "${WORKSPACE}/${SOFTWARE_DIR}" "${CATKIN_WS}/src/${SOFTWARE_DIR}"
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
catkin list
rosdep install --from-paths . --ignore-src --rosdistro=${ROS_DISTRO} --as-root apt:false 
echo '# END SECTION'

echo '# BEGIN SECTION compile the catkin workspace'
catkin build -j${MAKE_JOBS} --verbose --summary
echo '# END SECTION'

echo '# BEGIN SECTION: running tests'
catkin run_tests -j1 || true
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
DELIM_COMPILATION
