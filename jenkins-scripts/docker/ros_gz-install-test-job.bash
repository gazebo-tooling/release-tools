#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

[[ -z ${ENABLE_GZ_SIM_RUNTIME_TEST} ]] && ENABLE_GZ_SIM_RUNTIME_TEST=true

export GPU_SUPPORT_NEEDED=true

if [[ -z ${ROS_DISTRO} ]]; then
  echo "ROS_DISTRO is empty. Can not run test"
  exit 1
fi

export INSTALL_JOB_POSTINSTALL_HOOK="""
. /opt/ros/${ROS_DISTRO}/setup.bash

echo '# BEGIN SECTION: ros_gz talker/listener'
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg 2>/dev/null &
sleep 1
# ros2 listener
ros2 topic echo /chatter > /tmp/echo_chatter &
# gz transport talker
gz topic -t /chatter -m gz.msgs.StringMsg -p 'data:\"Hello\"' &
gz topic -i -t /chatter
if ! grep -q Hello /tmp/echo_chatter; then
  echo 'chatter log file does not contain the expected Hello string'
  exit 1
fi
echo '# END SECTION'

echo '# BEGIN SECTION: test gz_sim via ros2 launch'
# Workaround until ros_gz defines a dependency on gz-sim*-cli. Avoid to use
# apt --install-suggests since it installs the whole universe of packages
VER=\$(dpkg -l | grep libgz-sim | grep ^ii | head -1 | awk '{ print \$3 }')
sudo apt-get install -y gz-sim\${VER:0:1}-cli
TEST_START=\`date +%s\`
# preserve-status did not work here
timeout 180 ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf || true
TEST_END=\`date +%s\`
DIFF=\$((\$TEST_END-\$TEST_START))

if [ \$DIFF -lt 180 ]; then
 echo 'The test took less than 180s. Something bad happened'
 exit 1
fi
"""

export DEPENDENCY_PKGS="${DEPENDENCY_PKGS} wget ros-${ROS_DISTRO}-ros-base"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
