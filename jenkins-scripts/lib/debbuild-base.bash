#!/bin/bash -x

# DISTRO is passed as a jenkins job parameter. If not set up
# default to precise
if [ -z $DISTRO ]; then
    DISTRO=precise
fi;

# RELEASE_REPO_DIRECTORY control the migration from single distribution
# to multidistribution. If not set, go for ubuntu in single distribution
# mode
if [ -z $RELEASE_REPO_DIRECTORY ]; then
    RELEASE_REPO_DIRECTORY=ubuntu
fi;

NIGHTLY_MODE=false
if [ "${VERSION}" = "nightly" ]; then
    NIGHTLY_MODE=true
fi

###################################################
# Boilerplate.
# DO NOT MODIFY

#stop on error
set -e

distro=$DISTRO
arch=$ARCH
base=/var/cache/pbuilder-$distro-$arch

aptconffile=$WORKSPACE/apt.conf

#increment this value if you have changed something that will invalidate base tarballs. #TODO this will need cleanup eventually.
basetgz_version=2

rootdir=$base/apt-conf-$basetgz_version

basetgz=$base/base-$basetgz_version.tgz
output_dir=$WORKSPACE/output
work_dir=$WORKSPACE/work

sudo apt-get update
sudo apt-get install -y pbuilder python-empy python-argparse debhelper # todo move to server setup, or confirm it's there

if [ -e $WORKSPACE/catkin-debs ]
then
  rm -rf $WORKSPACE/catkin-debs
fi

git clone git://github.com/willowgarage/catkin-debs.git $WORKSPACE/catkin-debs -b master --depth 1


cd $WORKSPACE/catkin-debs
. setup.sh

#setup the cross platform apt environment
# using sudo since this is shared with pbuilder and if pbuilder is interupted it will leave a sudo only lock file.  Otherwise sudo is not necessary. 
# And you can't chown it even with sudo and recursive 
sudo PYTHONPATH=$PYTHONPATH $WORKSPACE/catkin-debs/scripts/setup_apt_root.py $distro $arch $rootdir --local-conf-dir $WORKSPACE --repo ros@http://packages.ros.org/ros/ubuntu

sudo rm -rf $output_dir
mkdir -p $output_dir

sudo rm -rf $work_dir
mkdir -p $work_dir
cd $work_dir

sudo apt-get update -c $aptconffile

# Grab a newer version of pbuilder, because the one that ships with Lucid suffers from a bug when using --execute
# https://bugs.launchpad.net/ubuntu/+source/pbuilder/+bug/811016
rm -f $WORKSPACE/pbuilder
wget -O $WORKSPACE/pbuilder http://bazaar.launchpad.net/~vcs-imports/pbuilder/trunk/download/head:/pbuilder/pbuilder
chmod a+x $WORKSPACE/pbuilder

# Setup the pbuilder environment if not existing, or update
if [ ! -e $basetgz ] || [ ! -s $basetgz ] 
then
  #make sure the base dir exists
  sudo mkdir -p $base
  #create the base image
  sudo $WORKSPACE/pbuilder create \
    --distribution $distro \
    --aptconfdir $rootdir/etc/apt \
    --basetgz $basetgz \
    --architecture $arch
else
  sudo $WORKSPACE/pbuilder --update --basetgz $basetgz
fi

# Boilerplate.
# DO NOT MODIFY
###################################################

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex

# Install deb-building tools
apt-get install -y pbuilder fakeroot debootstrap devscripts ubuntu-dev-tools mercurial debhelper reprepro wget

# get ROS repo's key, to be used in creating the pbuilder chroot (to allow it to install packages from that repo)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $DISTRO main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu $DISTRO main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Step 0: create/update distro-specific pbuilder environment
pbuilder-dist $DISTRO $ARCH create --othermirror "deb http://packages.ros.org/ros/ubuntu $DISTRO main|deb http://packages.osrfoundation.org/drc/ubuntu $DISTRO main" --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg

# Step 0: Clean up
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build

# Step 1: Get the source (nightly builds or tarball)
if ${NIGHTLY_MODE}; then
  apt-get install -y mercurial
  hg clone https://bitbucket.org/osrf/$PACKAGE
  PACKAGE_SRC_BUILD_DIR=$PACKAGE
else
  wget --quiet -O ${PACKAGE_ALIAS}_$VERSION.orig.tar.bz2 $SOURCE_TARBALL_URI
  rm -rf $PACKAGE-$VERSION
  tar xf ${PACKAGE_ALIAS}_$VERSION.orig.tar.bz2
  PACKAGE_SRC_BUILD_DIR=$PACKAGE-$VERSION
fi

# Step 4: add debian/ subdirectory with necessary metadata files to unpacked source tarball
rm -rf /tmp/$PACKAGE-release
hg clone https://bitbucket.org/osrf/$PACKAGE-release /tmp/$PACKAGE-release
cd /tmp/$PACKAGE-release
hg up $RELEASE_REPO_BRANCH

# Adding extra directories to code. debian has no problem but some extra directories 
# handled by symlinks (like cmake) in the repository can not be copied directly. 
# Need special care to copy, using first a --dereference
cd $WORKSPACE/build/\$PACKAGE_SRC_BUILD_DIR
cp -a --dereference /tmp/$PACKAGE-release/${RELEASE_REPO_DIRECTORY}/* .

# [nightly] Adjust version in nightly mode
if $NIGHTLY_MODE; then
  UPSTREAM_VERSION=1.4.0 # TODO fix this to get latest from changelog
  TIMESTAMP=\$(date '+%Y%m%d')
  REV=\$(hg parents --template="{node|short}\n")
  NIGHTLY_VERSION_SUFFIX=-\${UPSTREAM_VERSION}~hg\${TIMESTAMP}r\${REV}-${RELEASE_VERSION}~${ARCH}
  NIGHTLY_VERSION=${PACKAGE_ALIAS}-\${NIGHTLY_VERSION_SUFFIX}
  # Fix the changelog
  sed -i -e 's:xxxxx:\${NIGHTLY_VERSION}:g' debian/changelog
  # TODO: Fix CMakeLists.txt ?
fi

# Step 5: use debuild to create source package
#TODO: create non-passphrase-protected keys and remove the -uc and -us args to debuild
debuild -S -uc -us --source-option=--include-binaries

# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc

# Step 7: upload resulting .deb
sudo apt-get install -y openssh-client
cd /var/packages/gazebo/ubuntu

# Set proper package names
if $NIGHTLY_MODE; then
  PKG_NAME=\${NIGHTLY_VERSION}.deb
  DBG_PKG_NAME=${PACKAGE_ALIAS}-dbg\${NIGHTLY_VERSION_SUFFIX}.deb
else
  PKG_NAME=${PACKAGE_ALIAS}_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
  DBG_PKG_NAME=${PACKAGE_ALIAS}-dbg-${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
fi

MAIN_PKGS="/var/lib/jenkins/pbuilder/${DISTRO}-${ARCH}_result/\${PKG_NAME} /var/lib/jenkins/pbuilder/${DISTRO}_result/\${PKG_NAME}"
DEBUG_PKGS="/var/lib/jenkins/pbuilder/${DISTRO}-${ARCH}_result/\${DBG_PKG_NAME} /var/lib/jenkins/pbuilder/${DISTRO}_result/\${DBG_PKG_NAME}"

FOUND_PKG=0
for pkg in \${MAIN_PKGS}; do
    echo "looking for \$pkg"
    if [ -f \${pkg} ]; then
        echo "found \$pkg"
        FOUND_PKG=1
	ls -las \${pkg}
        break;
    fi
done
test \$FOUND_PKG -eq 1 || exit 1

FOUND_PKG=0
for pkg in \${DEBUG_PKGS}; do
    if [ -f \${pkg} ]; then
        # Check for correctly generated debug packages size > 2Kb
        # when not valid instructions in rules/control it generates 1.5K package
        test -z \$(find \$pkg -size +2k) && exit 1
	ls -las \${pkg}
        FOUND_PKG=1
        break;
    fi
done
test \$FOUND_PKG -eq 1 || echo "No debug packages found. No upload"
DELIM

# Copy in my GPG key, to allow reprepro to sign the debs it builds.
rm -rf $WORKSPACE/gnupg
cp -a $HOME/.gnupg $WORKSPACE/gnupg

# Copy in my ssh keys, to allow the above ssh/scp calls to work; not sure this is the best way to do it, 
# but it shouldn't be a security issue, as only Jenkins users can see the contents of the workspace
cp $HOME/.ssh/id_rsa $WORKSPACE
#
# Make project-specific changes here
###################################################

sudo $WORKSPACE/pbuilder  --execute \
    --bindmounts "$WORKSPACE /var/packages/gazebo/ubuntu" \
    --basetgz $basetgz \
    -- build.sh
