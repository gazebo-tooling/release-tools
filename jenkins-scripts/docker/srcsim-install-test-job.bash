#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

INSTALL_JOB_PREINSTALL_HOOK="""
# import the SRC repo
echo \"deb http://srcsim.gazebosim.org/src ${DISTRO} main\" >\\
                                           /etc/apt/sources.list.d/src.list
apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
wget -qO - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
sudo apt-get update
"""

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: testing by running qual1 launch file'
update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export IS_GAZEBO=true
export ROS_IP=127.0.0.1

chown -R \$USER:\$USER /opt/ros/indigo/share/ihmc_ros_java_adapter

mkdir -p ~/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ~/.ihmc/IHMCNetworkParameters.ini

echo '@ros - rtprio 99' > /etc/security/limits.d/ros-rtprio.conf'
groupadd ros
usermod -a -G ros \$USER

wget -P /tmp/ http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz
tar -xvf /tmp/valkyrie_controller.tar.gz -C \$HOME
rm /tmp/valkyrie_controller.tar.gz

mkdir ~/valkyrie

wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
mkdir -p ~/.gazebo/models
tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz

# pre-built cache
source  /opt/nasa/indigo/setup.bash
roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch

TEST_START=\`date +%s\`
timeout --preserve-status 400 roslaunch srcsim qual2.launch extra_gazebo_args:=\"-r\" init:=\"true\" walk_test:=true || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 400 ]; then
   echo 'The test took less than 400s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""
# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="wget bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
