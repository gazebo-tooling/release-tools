# Common instructions to create the building enviroment
set -e

# Important! GPU_SUPPORT_NEEDED to be use by the scripts.
# USE_GPU_DOCKER by internal lib/ scripts

[[ -z ${GPU_SUPPORT_NEEDED} ]] && GPU_SUPPORT_NEEDED=false

if ${GPU_SUPPORT_NEEDED}; then
    USE_GPU_DOCKER=true
fi

[[ -z $USE_GPU_DOCKER ]] && export USE_GPU_DOCKER=false

# Check disk space and if low:
#  *  Containers that exited more than 5 days ago are removed.
#  *  Images that don't belong to any remaining container after that are removed
if [[ -z ${DO_NOT_CHECK_DOCKER_DISK_USAGE} ]]; then
    # get the mount point of the docker directory, not always /
    docker_device=$(df '/var/lib/docker' | awk '{ print $1 }' | tail -n 1)
    # in seconds: 5 days = 432000s
    PERCENT_DISK_USED=$(df -h ${docker_device} | grep ${docker_device} | sed 's:.* \([0-9]*\)%.*:\1:')
    if [[ $PERCENT_DISK_USED -gt 90 ]]; then
        echo "Space left is low: ${PERCENT_DISK_USED}% used"
        echo "Run docker cleaner !!"
        wget https://raw.githubusercontent.com/spotify/docker-gc/master/docker-gc
        sudo bash -c "GRACE_PERIOD_SECONDS=432000 bash docker-gc"
    fi

    # if not enough, run again with 1 day = 86400s
    PERCENT_DISK_USED=$(df -h ${docker_device} | grep ${docker_device} | sed 's:.* \([0-9]*\)%.*:\1:')
    if [[ $PERCENT_DISK_USED -gt 90 ]]; then
        echo "Space left is still low: ${PERCENT_DISK_USED}% used"
        echo "Run docker cleaner !!"
        wget https://raw.githubusercontent.com/spotify/docker-gc/master/docker-gc
        sudo bash -c "GRACE_PERIOD_SECONDS=86400 bash docker-gc"
    fi

    # if not enough, kill the whole cache
    PERCENT_DISK_USED=$(df -h ${docker_device} | grep ${docker_device} | sed 's:.* \([0-9]*\)%.*:\1:')
    if [[ $PERCENT_DISK_USED -gt 90 ]]; then
        echo "Space left is low again: ${PERCENT_DISK_USED}% used"
        echo "Kill the whole docker cache !!"
        [[ -n $(sudo docker ps -q) ]] && sudo docker kill $(sudo docker ps -q) || true
        [[ -n $(sudo docker images -a -q) ]] && sudo docker rmi $(sudo docker images -a -q) || true
    fi
fi

# Timing
source ${SCRIPT_DIR}/../lib/boilerplate_timing_prepare.sh
init_stopwatch TOTAL_TIME
init_stopwatch CREATE_TESTING_ENVIROMENT

# Default values - Provide them is prefered
if [ -z ${DISTRO} ]; then
    DISTRO=xenial
fi

if [ -z ${ROS_DISTRO} ]; then
  ROS_DISTRO=kinetic
fi

if [ -z "${ROS2}" ]; then
  export ROS2=false
fi

if ${ROS2}; then
  export USE_COLCON=true
fi

# Define making jobs by default if not present
if [ -z ${MAKE_JOBS} ]; then
    MAKE_JOBS=1
fi

# Use reaper by default
if [ -z ${ENABLE_REAPER} ]; then
    ENABLE_REAPER=true
fi

# We use ignitionsrobotics or osrf. osrf by default
if [ -Z ${BITBUCKET_REPO} ]; then
    BITBUCKET_REPO="osrf"
fi

# By default, do not need to use C++11 compiler
if [ -z ${NEED_C11_COMPILER} ]; then
  NEED_C11_COMPILER=false
fi

if [ -z ${NEED_C17_COMPILER} ]; then
  NEED_C17_COMPILER=false
fi

# By default, do not use ROS
if [ -z ${ENABLE_ROS} ]; then
  ENABLE_ROS=false
fi

# Transition for 4.8 -> 4.9 makes some optimization in the linking
# which can break some software. Use it as a workaround in this case
if [ -z ${NEED_GCC48_COMPILER} ]; then
  NEED_GCC48_COMPILER=false
fi

# Only precise needs to install a C++11 compiler. Trusty on
# already have a supported version
if $NEED_C11_COMPILER; then
  if [[ $DISTRO != 'precise' ]]; then
      NEED_C11_COMPILER=false
  fi
fi

# in some machines squid is returning
[ -z ${NEED_SQUID_WORKAROUND} ] && NEED_SQUID_WORKAROUND=false

# Useful for running tests properly integrated ros based software
if ${ENABLE_ROS}; then
  export USE_ROS_REPO=true
  export ROS_HOSTNAME=localhost
  export ROS_MASTER_URI=http://localhost:11311
  export ROS_IP=127.0.0.1
fi

if [[ -n `ps aux | grep gzserver | grep -v grep` ]]; then
    echo "There is a gzserver already running on the machine. Stopping"
    exit -1
fi

. ${SCRIPT_DIR}/../lib/check_graphic_card.bash
. ${SCRIPT_DIR}/../lib/dependencies_archive.sh

# After dependencies_archive.sh call to get the rest of BASE_DEPENDENCIES
if [ -z "${ENABLE_CCACHE}" ]; then
  ENABLE_CCACHE=true
  BASE_DEPENDENCIES="${BASE_DEPENDENCIES} ccache"
  CCACHE_DIR="/srv/ccache"
  CCACHE_MAXSIZE=${CCACHE_MAXSIZE:-5G}
  # create the host cache dir to be shared across all docker images
  if [[ ! -d ${CCACHE_DIR} ]]; then
    sudo mkdir -p ${CCACHE_DIR}
    sudo chmod o+w ${CCACHE_DIR}
  fi
fi


output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

# TODO: Check for docker package
NEEDED_HOST_PACKAGES="mercurial python-setuptools python-psutil qemu-user-static gpgv squid-deb-proxy"
# python-argparse is integrated in libpython2.7-stdlib since raring
# Check for precise in the HOST system (not valid DISTRO variable)
if [[ $(lsb_release -sr | cut -c 1-5) == '12.04' ]]; then
    NEEDED_HOST_PACKAGES="${NEEDED_HOST_PACKAGES} python2.7"
else
    NEEDED_HOST_PACKAGES="${NEEDED_HOST_PACKAGES} libpython2.7-stdlib"
fi

# Check if they are already installed in the host.
# dpkg-query will return an error in stderr if a package has never been in the
# system. It will return a header composed by several lines started with |, +++
# and 'Desired' the rest of lines is composed by: ^rc or ^un if the package is
# not in the system. ^in if it is installed
QUERY_RESULT=$(dpkg-query --list ${NEEDED_HOST_PACKAGES} 2>&1 | grep -v ^ii | grep -v '|' | grep -v '^\+++' | grep -v '^Desired') || true
if [[ -n ${QUERY_RESULT} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

# Check that all of them are present in the system, not returning false
if [[ ! $(dpkg-query --list ${NEEDED_HOST_PACKAGES}) ]]; then
  echo "Some needed packages are failing in the host"
  exit 1
fi

# Check if squid-deb-proxy is running or start it otherwise
if [[ -z $(ps aux | grep squid-deb-proxy.conf | grep -v grep | awk '{ print $2}') ]]; then
  sudo service squid-deb-proxy start
fi

if ${NEED_SQUID_WORKAROUND}; then
  sudo service squid-deb-proxy restart
fi

# Docker checking
# Code imported from https://github.com/CognitiveRobotics/omnimapper/tree/master/docker
# under the license detailed in https://github.com/CognitiveRobotics/omnimapper/blob/master/LICENSE
#version_gt() {
#    test "$(echo "$@" | tr " " "\n" | sort -V | tail -n 1)" == "$1";
#}

#docker_version=$(docker version | grep 'Client version' | awk '{split($0,a,":"); print a[2]}' | tr -d ' ')

# Docker 1.3.0 or later is required for --device
#if ! version_gt "${docker_version}" "1.2.0"; then
#  echo "Docker version 1.3.0 or greater is required"
#  exit 1
#fi

# CID file to create
# - In jenkins we use PROJECT_NAME + BUILD_NUMBER
# Check if the job define a DOCKER_JOB_NAME or generate one random
DOCKER_RND_ID=$(( ( RANDOM % 10000 )  + 1 ))

if [[ -z $DOCKER_JOB_NAME ]]; then
    export DOCKER_JOB_NAME=${DOCKER_RND_ID}
    echo "Warning: DOCKER_JOB_NAME was not defined"
    echo " - using ${DOCKER_JOB_NAME}"
fi

# Check if the job was called from jenkins
if [[ -n ${BUILD_NUMBER} ]]; then
   export DOCKER_JOB_NAME="${DOCKER_JOB_NAME}:${BUILD_NUMBER}"
else
   # Reuse the random id
   export DOCKER_JOB_NAME="${DOCKER_JOB_NAME}:${DOCKER_RND_ID}"
fi

echo " - Using DOCKER_JOB_NAME ${DOCKER_JOB_NAME}"

export CIDFILE="${WORKSPACE}/${DOCKER_JOB_NAME}.cid"

# CIDFILE should not exists
if [[ -f ${CIDFILE} ]]; then
    echo "CIDFILE: ${CIDFILE} exists, which will make docker to fail."
    echo "Container ID file found, make sure the other container isn't running"
    exit 1
fi

export DOCKER_TAG="${DOCKER_JOB_NAME}"

# It is used to invalidate cache
TODAY_STR=$(date +%D)
MONTH_YEAR_STR=$(date +%m%y)

# Clean previous results in the workspace if any
if [[ -z ${KEEP_WORKSPACE} ]]; then
    # Clean previous results, need to next mv command not to fail
    for d in $(find ${WORKSPACE} -maxdepth 1 -name '*_results' -type d); do
        sudo rm -fr ${d}
    done
fi
# Cleanup dockerfile and build.sh if exists from previous runs
rm -fr Dockerfile
rm -fr ${WORKSPACE}/build.sh

cd ${WORKSPACE}
