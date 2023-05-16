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
echo '# BEGIN SECTION: test gz_sim via ros2 launch'
. /opt/ros/${ROS_DISTRO}/setup.bash
# TEST_START=\`date +%s\`
# timeout --preserve-status 180 ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf
# TEST_END=\`date +%s\`
# DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

# if [ \$DIFF -lt 180 ]; then
#   echo 'The test took less than 180s. Something bad happened'
#   exit 1
# fi
# echo '# END SECTION'
echo '# BEGIN SECTION: ros_gz talker/listener'
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg 2 > /dev/null \&
sleep 1
ros2 topic echo /chatter > /tmp/echo_chatter \&
sleep 1
gz topic -t /chatter -m gz.msgs.StringMsg -p 'data:\"Hello\"' \&
sleep 1
if [[ -z \$(cat /tmp/echo_chatter) ]]; then
  echo 'chatter log file is empty'
  exit 1
fi
echo '# END SECTION'
"""

# Need bc to proper testing and parsing the time
# TODO: fix gz-sim-cli dependency on ros_gz package
# to avoid hardcoded version on gz-sim
export DEPENDENCY_PKGS="${DEPENDENCY_PKGS} wget bc ros-${ROS_DISTRO}-ros-base gz-sim7-cli"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
