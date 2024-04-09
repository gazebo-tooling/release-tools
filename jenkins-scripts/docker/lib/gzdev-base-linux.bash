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
# pip3 install -r requirements.txt
echo '# END SECTION'

echo '# BEGIN SECTION: smoke tests for ign-docker-env'
sudo pip3 install git+https://github.com/adlarkin/ign-rocker.git

# TODO: rocker can not play well docker-in-docker installations
# out of the box. Needs more work.

# xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
# export display=:1.0
# export mesa_gl_version_override=3.3

# test_timeout=300
# test_start=\`date +%s\`
# sudo bash -c "timeout --preserve-status \$test_timeout ./gzdev.py ign-docker-env dome --linux-distro ubuntu:bionic"
# test_end=\`date +%s\`
# diff=\$(expr \$test_end - \$test_start)

# if [ \$diff -lt \$test_timeout ]; then
#    echo 'the test took less than \$test_timeout. something bad happened'
#    exit 1
# fi
echo '# END SECTION'
DELIM

export USE_DOCKER_IN_DOCKER=true
export OSRF_REPOS_TO_USE="stable"
export USE_ROS_REPO=true
export ROS2=true
export DEPENDENCY_PKGS="python3-pip \
                 bash \
                 apt-transport-https \
                 ca-certificates \
                 curl \
                 software-properties-common \
                 python3-rocker \
                 psmisc \
                 xvfb"

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
