#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

export GPU_SUPPORT_NEEDED=true

export INSTALL_JOB_PKG="ariac"
export INSTALL_JOB_REPOS="stable"
export USE_ROS_REPO=true

case "$DISTRO" in
  'trusty')
    export ROS_DISTRO=indigo
    ;;
  'xenial')
    export ROS_DISTRO=kinetic
    ;;
  *)
    echo "Only trusty+indigo and xenial+kinetic are supported"
    exit 1
    ;;
esac

INSTALL_JOB_POSTINSTALL_HOOK="""
echo '# BEGIN SECTION: testing by running ariac launch file'
. /opt/ros/$ROS_DISTRO/setup.bash
rosdep init
rosdep update

apt-get install -y wget

# Follow tutorial: http://wiki.ros.org/ariac/Tutorials/HelloWorld
mkdir -p ~/helloworld_ws/src/ariac_example
cd ~/helloworld_ws/src/ariac_example
wget https://bitbucket.org/osrf/ariac/raw/master/ariac_example/package.xml 
wget https://bitbucket.org/osrf/ariac/raw/no_catkin_python_setup/ariac_example/CMakeLists.txt

mkdir -p ~/helloworld_ws/src/ariac_example/config
cd ~/helloworld_ws/src/ariac_example/config
wget https://bitbucket.org/osrf/ariac/raw/master/ariac_example/config/sample_gear_conf.yaml

mkdir -p ~/helloworld_ws/src/ariac_example/src
cd ~/helloworld_ws/src/ariac_example/src
wget https://bitbucket.org/osrf/ariac/raw/master/ariac_example/src/ariac_example_node.cpp

cd ~/helloworld_ws
catkin_make

TEST_START=\`date +%s\`
# Run ARIAC simulation
rosrun osrf_gear gear.py -f \$(rospack find osrf_gear)/config/comp_conf1.yaml \
    ~/helloworld_ws/src/ariac_example/config/sample_gear_conf.yaml &
# Run the example node
sleep 10 && . ~/helloworld_ws/devel/setup.bash && \
   timeout --preserve-status 180 rosrun ariac_example ariac_example_node || true

TEST_END=\`date +%s\`
DIFF=\`echo \"\$TEST_END - \$TEST_START\" | bc\`

# Kill the simulation
kill %%
if [ $? != 0 ]; then
   echo 'The ARIAC simulation had already died when trying to kill it. Something bad happened'
   exit 1
fi

# Check the test run-time
if [ \$DIFF -lt 180 ]; then
   echo 'The test took less than 180s. Something bad happened'
   exit 1
fi
echo '# END SECTION'
"""

. ${SCRIPT_DIR}/lib/generic-install-base.bash
