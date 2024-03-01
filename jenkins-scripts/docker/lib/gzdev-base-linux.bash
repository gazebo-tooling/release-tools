echo '# BEGIN SECTION: setup the testing enviroment'
export DOCKER_JOB_NAME="gzdev"
. "${SCRIPT_DIR}/lib/boilerplate_prepare.sh"
echo '# END SECTION'

. ${SCRIPT_DIR}/lib/_install_nvidia_docker.sh

cat > build.sh << DELIM
###################################################
#
set -ex

export MAKE_JOBS=${MAKE_JOBS}
export DISPLAY=${DISPLAY}

${INSTALL_NVIDIA_DOCKER2}

echo '# BEGIN SECTION: install pip requirements'
cd ${WORKSPACE}/gzdev
pip3 install -r requirements.txt
echo '# END SECTION'

echo '# BEGIN SECTION: smoke tests for ign-docker-env'
pip3 install wheel
pip3 install rocker
pip3 install git+https://github.com/adlarkin/ign-rocker.git

TEST_TIMEOUT=300
TEST_START=\`date +%s\`
timeout --preserve-status \$TEST_TIMEOUT ./gzdev.py ign-docker-env dome --linux-distro ubuntu:bionic
TEST_END=\`date +%s\`
DIFF=\$(expr \$TEST_END - \$TEST_START)

if [ \$DIFF -lt \$TEST_TIMEOUT ]; then
   echo 'The test took less than \$TEST_TIMEOUT. Something bad happened'
   exit 1
fi
echo '# END SECTION'
DELIM

export USE_DOCKER_IN_DOCKER=true
export OSRF_REPOS_TO_USE="stable"
export USE_ROS_REPO=true
export DEPENDENCY_PKGS="python3-pip \
                 bash \
                 apt-transport-https \
                 ca-certificates \
                 curl \
                 software-properties-common \
                 python3-rocker \
                 psmisc" # killall

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
