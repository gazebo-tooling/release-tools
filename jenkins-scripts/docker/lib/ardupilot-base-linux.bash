#!/bin/bash -x

echo '# BEGIN SECTION: setup the testing enviroment'
DOCKER_JOB_NAME="ardupilot_ci"
. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

echo '#BEGIN SECTION: install special branch of gazebo'
mkdir -p ${WORKSPACE}/gazebo/build
hg clone https://bitbucket.org/osrf/gazebo -b aero_apm_irlock ${WORKSPACE}/gazebo
cd ${WORKSPACE}/gazebo/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/usr -DENABLE_TESTS_COMPILATION=False ..
make -j${MAKE_JOBS}
make install
echo '#END SECTION'

echo '#BEGIN SECTION: install special branch of gazebo models'
mkdir -p \$HOME/.gazebo/models
mkdir -p ${WORKSPACE}/models/build
hg clone https://bitbucket.org/osrf/gazebo_models -b aero_testing_john ${WORKSPACE}/models
cp -r ${WORKSPACE}/models/gazebo_models/iris \${HOME}/.gazebo/models
cp -r ${WORKSPACE}/models/gazebo_models/gimbal_small_2d \${HOME}/.gazebo/models
echo '#END SECTION'

echo '# BEGIN SECTION: configuring'
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build
echo '# END SECTION'

echo '# BEGIN SECTION: compiling'
make -j${MAKE_JOBS}
echo '# END SECTION'

echo '# BEGIN SECTION: installing'
make install
echo '# END SECTION'

pip install MAVProxy
pip install dronekit

. /usr/share/gazebo/setup.sh
export PATH=\$PATH:${WORKSPACE}/ardupilot/Tools/autotest

echo '# BEGIN SECTION: running tests'
mkdir -p ${WORKSPACE}/gazebo_apm_sitl
hg clone https://bitbucket.org/iche033/gazebo_apm_sitl ${WORKSPACE}/gazebo_apm_sitl
mkdir -p $WORKSPACE/gazebo_apm_sitl/build
cd $WORKSPACE/gazebo_apm_sitl/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j${MAKE_JOBS}
echo '# END SECTION'
DELIM

SOFTWARE_DIR="ardupilot"
OSRF_REPOS_TO_USE="stable"
DEPENDENCY_PKGS="${BASE_DEPENDENCIES} \
                 ${GAZEBO_BASE_DEPENDENCIES} \
		 ${GAZEBO_EXTRA_DEPENDENCIES} \
		 python-setuptools \
		 python-dev \
		 python-opencv \
		 python-pip \
		 python-matplotlib \
		 python-pygame \
		 python-lxml \
		 ssh"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
