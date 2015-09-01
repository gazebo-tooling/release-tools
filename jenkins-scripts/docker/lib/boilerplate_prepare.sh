# Common instructions to create the building enviroment
set -e

# GPU_SUPPORT_NEEDED to be use by the scripts.
# USE_GPU_DOCKER by internal lib/ scripts

[[ -z ${GPU_SUPPORT_NEEDED} ]] && GPU_SUPPORT_NEEDED=false
   
if ${GPU_SUPPORT_NEEDED}; then
    USE_GPU_DOCKER=true
fi

[[ -z $USE_GPU_DOCKER ]] && export USE_GPU_DOCKER=false

# Default values - Provide them is prefered
if [ -z ${DISTRO} ]; then
    DISTRO=trusty
fi

if [ -z ${ROS_DISTRO} ]; then
  ROS_DISTRO=hydro
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

# Useful for running tests properly in ros based software
if ${ENABLE_ROS}; then
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

output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

NEEDED_HOST_PACKAGES="mercurial docker.io python-setuptools python-psutil qemu-user-static gpgv"
# python-argparse is integrated in libpython2.7-stdlib since raring
# Check for precise in the HOST system (not valid DISTRO variable)
if [[ $(lsb_release -sr | cut -c 1-5) == '12.04' ]]; then
    NEEDED_HOST_PACKAGES="${NEEDED_HOST_PACKAGES} python2.7"
else
    NEEDED_HOST_PACKAGES="${NEEDED_HOST_PACKAGES} libpython2.7-stdlib"
fi

# Check if they are already installed in the host
QUERY_HOST_PACKAGES=$(dpkg-query --list ${NEEDED_HOST_PACKAGES} | grep '^un ') || true
if [[ -n ${QUERY_HOST_PACKAGES} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

# Some packages will not show as ^un in the previous query but will return false if
# they are not present
if [[ ! $(dpkg-query --list ${NEEDED_HOST_PACKAGES}) ]]; then
  echo "Some needed packages are failing in the host"
  exit 1
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
    for d in $(find ${WORKSPACE} -name '*_results' -type d); do
        sudo rm -fr ${d}
    done
fi

rm -fr Dockerfile
cd ${WORKSPACE}
