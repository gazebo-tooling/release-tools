#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true
export USE_DOCKER_IN_DOCKER=true
# squid is blocking nvidia packages inside the dockerhub image
export USE_SQUID=false
export TIMEOUT=${TIMEOUT:-180}

. ${SCRIPT_DIR}/lib/_install_nvidia_docker.sh

export ROS_SETUP_POSTINSTALL_HOOK="""

${INSTALL_NVIDIA_DOCKER2}

xhost +

cd ${WORKSPACE}/subt/docker/
sed -i -e s:--uid\\ \\$\\{user_id\\}:: cloudsim_sim/Dockerfile
sed -i -e s:--uid\\ \\$\\{user_id\\}:: cloudsim_bridge/Dockerfile
sed -i -e s:-it:: run.bash

chmod +x build.bash run.bash join.bash
echo '# BEGIN SECTION: build cloudsim_sim'
bash -xe ./build.bash cloudsim_sim --no-cache
echo '# END SECTION'
echo '# BEGIN SECTION: build cloudsim_bridge'
bash -xe ./build.bash cloudsim_bridge --no-cache
echo '# END SECTION'

echo '# BEGIN SECTION: run cloudsim_sim'
bash -xe ./run.bash cloudsim_sim cloudsim_sim.ign robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG1 robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG2 &
sleep 5m
echo '# END SECTION'
echo '# BEGIN SECTION: run cloudsim_bridge'
bash -xe ./run.bash cloudsim_bridge cloudsim_bridge.ign robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG1 robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG2 &
sleep 5m
echo '# END SECTION'
"""

# The following instructions can not be tested since there is a network problem
# in the Jenkins setup
#
# [X1] CommsClient::Register: Problem registering with broker
# [CommsClient] Retrying register..

# roslaunch subt_example example_robot.launch name:=X1 &
# sleep 2m
# roslaunch subt_example example_robot.launch name:=X2 &
# sleep 2m
# timeout --preserve-status 10 rostopic pub /X2/comm std_msgs/String 'X1'
# timeout --preserve-status 10 rostopic pub /X1/cmd_vel geometry_msgs/Twist '{linear:  {x: 0.1, y: 0.0, z: 0.0}}'


export DEPENDENCY_PKGS="${DEPENDENCY_PKGS} software-properties-common xauth x11-xserver-utils"

. ${SCRIPT_DIR}/lib/subt-compilation-base.bash
