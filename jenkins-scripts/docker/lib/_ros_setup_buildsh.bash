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

cat >> build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

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

# Create the catkin workspace
rm -fr ${CATKIN_WS}/src
mkdir -p ${CATKIN_WS}/src
cd ${CATKIN_WS}/src
catkin_init_workspace
cp -a "${WORKSPACE}/${SOFTWARE_DIR}" .
cd ${CATKIN_WS}
catkin_make -j${MAKE_JOBS}
SHELL=/bin/sh . $WORKSPACE/ws/devel/setup.sh
DELIM
