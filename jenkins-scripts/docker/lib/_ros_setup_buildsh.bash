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

cat > build.sh << DELIM
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
rm -fr ${CATKIN_WS}/src
mkdir -p ${CATKIN_WS}/src
catkin config --init --mkdirs
cd ${CATKIN_WS}/src
rospack profile
ln -s "${WORKSPACE}/${SOFTWARE_DIR}" .
cd ${CATKIN_WS}
catkin build -j${MAKE_JOBS} --verbose --sumary
echo '# END SECTION'

echo '# BEGIN SECTION: running tests'
catkin run_tests -j1 || true
catkin_test_results

# link test results to usual place
mv ${CATKIN_WS}/build/test_results ${WORKSPACE}/build/
echo '# END SECTION'
DELIM
