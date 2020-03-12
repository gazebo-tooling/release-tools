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

${INSTALL_NVIDIA_DOCKER1}

echo '# BEGIN SECTION: install pip requirements'
cd ${WORKSPACE}/gzdev
pip3 install -r requirements.txt
echo '# END SECTION'

echo '# BEGIN SECTION: run gzdev for gazebo8 with nvidia'
cd ${WORKSPACE}/gzdev
./gzdev.py spawn --gzv=8 --nvidia
echo '# END SECTION'

echo '# BEGIN SECTION: Disply log file.'
cat ./gz8.log
echo '# END SECTION'

echo '# BEGIN SECTION: check that gazebo is running'
gazebo_detection=false
seconds_waiting=0
cat gz8.log | grep "Connected to gazebo master" && gazebo_detection=true
while (! \$gazebo_detection); do
   sleep 1
   docker top gz8 | grep gazebo && gazebo_detection=true
   docker top gz8 | grep gzserver && gazebo_detection=true
   seconds_waiting=\$((seconds_waiting+1))
   [ \$seconds_waiting -gt 30 ] && break
done
# clean up gazebo instances
docker rm -f gz8 || true
killall -9 gazebo gzserver gzclient || true
! \${gazebo_detection} && exit 1
echo '# END SECTION'
DELIM

export USE_DOCKER_IN_DOCKER=true
export OSRF_REPOS_TO_USE="stable"
export DEPENDENCY_PKGS="python3-pip \
                 bash \
                 apt-transport-https \
                 ca-certificates \
                 curl \
                 software-properties-common \
		 psmisc" # killall

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
