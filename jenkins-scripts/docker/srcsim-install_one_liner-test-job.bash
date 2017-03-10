#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

# Import library
. ${SCRIPT_DIR}/lib/_srcsim_lib.bash

# Both empty, the one line script should handle all the stuff
export INSTALL_JOB_PKG=""
export INSTALL_JOB_REPOS=""

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: run the one-liner installation'
export USER=root
# curl -ssL http://get.srcsim.gazebosim.org | sh
# TODO: temporary testing method
sh ${WORKSPACE}/scripts/one-line-installations/srcsim.sh
echo '# END SECTION'

echo '# BEGIN SECTION: testing by running qual2 launch file'
# TODO: remove when setup.bash from srcsim is merged
update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java
update-alternatives --set javac /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/javac
rm /usr/lib/jvm/default-java
sudo ln -s /usr/lib/jvm/java-8-openjdk-amd64 /usr/lib/jvm/default-java
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export IS_GAZEBO=true
export ROS_IP=127.0.0.1

mkdir -p ~/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ~/.ihmc/IHMCNetworkParameters.ini

TEST_START=\`date +%s\`
timeout --preserve-status 400 roslaunch srcsim qual2.launch extra_gazebo_args:=\"-r\" init:=\"true\" walk_test:=true || true
TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

if [ \$DIFF -lt 800 ]; then
   echo 'The test took less than 800s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="bc"

. ${SCRIPT_DIR}/lib/generic-install-base.bash
