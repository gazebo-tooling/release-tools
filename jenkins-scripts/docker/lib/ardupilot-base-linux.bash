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
export GZ_BRANCH=aero_apm_irlock
if [[ -d ${WORKSPACE}/gazebo ]]; then
  cd $WORKSPACE/gazebo
  hg pull
  hg up \$GZ_BRANCH --clean
else
  rm -fr ${WORKSPACE}/gazebo
  hg clone https://bitbucket.org/osrf/gazebo -b \$GZ_BRANCH ${WORKSPACE}/gazebo
  mkdir -p ${WORKSPACE}/gazebo/build
fi
cd ${WORKSPACE}/gazebo/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/usr -DENABLE_TESTS_COMPILATION=False ..
make -j${MAKE_JOBS}
make install
echo '#END SECTION'

echo '#BEGIN SECTION: install special branch of gazebo models'
export GZ_MODEL_BRANCH=aero_testing_john
mkdir -p \$HOME/.gazebo/models
if [[ -d ${WORKSPACE}/models ]]; then
  cd ${WORKSPACE}/models
  hg pull
  hg up \$GZ_MODEL_BRANCH --clean
else
  rm -fr ${WORKSPACE}/models
  hg clone https://bitbucket.org/osrf/gazebo_models -b \$GZ_MODEL_BRANCH ${WORKSPACE}/models
fi
cp -r ${WORKSPACE}/models/iris_with_standoffs \${HOME}/.gazebo/model
cp -r ${WORKSPACE}/models/gimbal_small_2d \${HOME}/.gazebo/models
echo '#END SECTION'

pip install MAVProxy
pip install dronekit

. /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:\$HOME/.gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\$HOME/.gazebo/models
export PATH=\$PATH:${WORKSPACE}/ardupilot/Tools/autotest

echo '# BEGIN SECTION: running tests'
rm -fr ${WORKSPACE}/gazebo_apm_sitl
hg clone https://bitbucket.org/iche033/gazebo_apm_sitl ${WORKSPACE}/gazebo_apm_sitl
mkdir -p $WORKSPACE/gazebo_apm_sitl/build
cd $WORKSPACE/gazebo_apm_sitl/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j${MAKE_JOBS}
make test ARGS="-VV
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
		 mercurial \
		 ssh"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
