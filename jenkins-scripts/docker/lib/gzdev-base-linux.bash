echo '# BEGIN SECTION: setup the testing enviroment'
export DOCKER_JOB_NAME="gzdev"
. "${SCRIPT_DIR}/lib/boilerplate_prepare.sh"
. "${SCRIPT_DIR}/lib/_common_scripts.bash"
. "${SCRIPT_DIR}/lib/_install_nvidia_docker.sh"
echo '# END SECTION'

cat > build.sh << DELIM
$(generate_buildsh_header)

export MAKE_JOBS=${MAKE_JOBS}
export DISPLAY=${DISPLAY}

${INSTALL_NVIDIA_DOCKER2}

echo '# BEGIN SECTION: install pip requirements'
cd ${WORKSPACE}/gzdev
# pip3 install -r requirements.txt
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
