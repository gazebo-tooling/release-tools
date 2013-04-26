# Common instructions to create the building enviroment
set -e

# Default values - Provide them is prefered
if [ -z ${DISTRO} ]; then
    DISTRO=precise
fi

if [ -z ${ROS_DISTRO} ]; then
  ROS_DISTRO=fuerte
fi

. ${SCRIPT_DIR}/lib/check_graphic_card.bash
. ${SCRIPT_DIR}/lib/dependencies_archive.sh

distro=${DISTRO}
arch=amd64
base=/var/cache/pbuilder-$distro-$arch

aptconffile=$WORKSPACE/apt.conf

#increment this value if you have changed something that will invalidate base tarballs. #TODO this will need cleanup eventually.
basetgz_version=2

rootdir=$base/apt-conf-$basetgz_version

basetgz=$base/base-$basetgz_version.tgz
output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

NEEDED_HOST_PACKAGES="pbuilder python-empy python-argparse debhelper"
# Check if they are already installed in the host
QUERY_HOST_PACKAGES=$(dpkg-query -Wf'${db:Status-abbrev}' ${NEEDED_HOST_PACKAGES} 2>&1) || true
if [[ -n ${QUERY_HOST_PACKAGES} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

#setup the cross platform apt environment
# using sudo since this is shared with pbuilder and if pbuilder is interupted it will leave a sudo only lock file.  Otherwise sudo is not necessary. 
# And you can't chown it even with sudo and recursive
cd $WORKSPACE/scripts/catkin-debs/
sudo ./setup_apt_root.py $distro $arch $rootdir --local-conf-dir $WORKSPACE --repo ros@http://packages.ros.org/ros/ubuntu

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
    --architecture $arch
else
  sudo pbuilder --update --basetgz $basetgz
fi
