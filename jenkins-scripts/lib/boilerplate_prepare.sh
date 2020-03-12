# Common instructions to create the building enviroment
set -e

# Timing
source ${SCRIPT_DIR}/lib/boilerplate_timing_prepare.sh
init_stopwatch TOTAL_TIME
init_stopwatch CREATE_TESTING_ENVIROMENT

# Default values - Provide them is prefered
if [ -z ${DISTRO} ]; then
    DISTRO=xenial
fi

if [ -z ${ROS_DISTRO} ]; then
  ROS_DISTRO=kinetic
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

. ${SCRIPT_DIR}/lib/check_graphic_card.bash
. ${SCRIPT_DIR}/lib/dependencies_archive.sh

# Workaround for precise pbuilder-dist segfault
# https://bitbucket.org/osrf/release-tools/issue/22
if [[ -z $WORKAROUND_PBUILDER_BUG ]]; then
  WORKAROUND_PBUILDER_BUG=false
fi

distro=${DISTRO}

if [ -z "${ARCH+xxx}" ]; then
    export ARCH=amd64
fi

arch=${ARCH}
base=/var/cache/pbuilder-$distro-$arch
aptconffile=$WORKSPACE/apt.conf

#increment this value if you have changed something that will invalidate base tarballs. #TODO this will need cleanup eventually.
basetgz_version=2

rootdir=$base/apt-conf-$basetgz_version

basetgz=$base/base-$basetgz_version.tgz
output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

NEEDED_HOST_PACKAGES="mercurial pbuilder python-empy debhelper python-setuptools python-psutil gpgv"
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

# monitor all subprocess and enforce termination (thanks to ROS crew)
# never failed on this
if $ENABLE_REAPER; then
# Hack for not failing when github is down
download_done=false
seconds_waiting=0
while (! $download_done); do
  wget https://raw.github.com/ros-infrastructure/buildfarm/master/scripts/subprocess_reaper.py -O subprocess_reaper.py && download_done=true
  sleep 1
  seconds_waiting=$((seconds_waiting+1))
  [ $seconds_waiting -gt 60 ] && exit 1
done

sudo python subprocess_reaper.py $$ &
sleep 1
fi

#setup the cross platform apt environment
# using sudo since this is shared with pbuilder and if pbuilder is interupted it will leave a sudo only lock file.  Otherwise sudo is not necessary. 
# And you can't chown it even with sudo and recursive
cd $WORKSPACE/scripts/catkin-debs/

ubuntu_repo_url="http://us.archive.ubuntu.com/ubuntu"

# If using a depracted distro, you need to use old-releases from ubuntu
if [[ $DISTRO == 'raring' ]]; then
  ubuntu_repo_url="http://old-releases.ubuntu.com/ubuntu/"
fi

if $ENABLE_ROS; then
  ros_repository_str="--repo ros@http://packages.ros.org/ros/ubuntu"
fi

sudo ./setup_apt_root.py $distro $arch $rootdir \
                          --mirror $ubuntu_repo_url $ros_repository_str \
			  --local-conf-dir $WORKSPACE 
sudo rm -rf $output_dir
mkdir -p $output_dir

sudo rm -rf $work_dir
mkdir -p $work_dir
cd $work_dir

sudo apt-get update -c $aptconffile

# Setup the pbuilder environment if not existing, or update
if [ ! -e $basetgz ] || [ ! -s $basetgz ] 
then
  #make sure the base dir exists
  sudo mkdir -p $base
  #create the base image
  sudo pbuilder create \
    --distribution $distro \
    --aptconfdir $rootdir/etc/apt \
    --basetgz $basetgz \
    --architecture $arch \
    --mirror $ubuntu_repo_url
else
  sudo pbuilder --update --basetgz $basetgz --mirror $ubuntu_repo_url
fi
