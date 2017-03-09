#!/bin/bash -x

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

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

PACKAGE_UNDERSCORE_NAME=${PACKAGE//-/_}

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
#!/usr/bin/env bash
set -ex

# ccache is sometimes broken and has now reason to be used here
# http://lists.debian.org/debian-devel/2012/05/msg00240.html
echo "unset CCACHEDIR" >> /etc/pbuilderrc
PBUILD_DIR=\$HOME/.pbuilder
mkdir -p \$PBUILD_DIR

# Install deb-building tools
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools mercurial git debhelper wget cdbs 

# get ROS repo's key, to be used in creating the pbuilder chroot (to allow it to install packages from that repo)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $DISTRO main" > /etc/apt/sources.list.d/ros-latest.list'
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu $DISTRO main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Hack to avoid problem with non updated 
if [ $DISTRO = 'precise' ]; then
  echo "Skipping pbuilder check for outdated info"
  #sed -i -e 's:UbuntuDistroInfo().devel():self.target_distro:g' /usr/bin/pbuilder-dist
fi

# Step 0: create/update distro-specific pbuilder environment
OTHERMIRROR='deb http://packages.ros.org/ros/ubuntu $DISTRO main|deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main' pbuilder-dist $DISTRO $ARCH create --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg

# Step 0: Clean up
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build

# Step 4: checkout bloom software -release repo 
rm -rf /tmp/$PACKAGE-release
git clone ${UPSTREAM_RELEASE_REPO} /tmp/$PACKAGE-release
cd /tmp/$PACKAGE-release

FULL_VERSION=$VERSION-$RELEASE_VERSION
FULL_DEBIAN_BRANCH_NAME=ros-$ROS_DISTRO-$PACKAGE\_\$FULL_VERSION\_$DISTRO

git checkout release/$ROS_DISTRO/$PACKAGE_UNDERSCORE_NAME/\$FULL_VERSION
git checkout debian/\$FULL_DEBIAN_BRANCH_NAME

if [ -f debian/control ]; then
cat debian/control
fi

echo | dh_make -s --createorig -p ros-$ROS_DISTRO-${PACKAGE}_${VERSION} || true
ls ..

# Step 5: use debuild to create source package
#TODO: create non-passphrase-protected keys and remove the -uc and -us args to debuild
debuild --no-tgz-check -S -uc -us --source-option=--include-binaries -j${MAKE_JOBS}

cat > \$PBUILD_DIR/A10_run_rosdep << DELIM_ROS_DEP
#!/bin/sh
if [ -f /usr/bin/rosdep ]; then
  # root share the same /tmp/buildd HOME than pbuilder user. Need to specify the root
  # HOME=/root otherwise it will make cache created during ros call forbidden to 
  # rosdep needs certificates to access to https
  apt-get install -y python-openssl ca-certificates
  HOME=/root rosdep init
fi
DELIM_ROS_DEP
chmod a+x \$PBUILD_DIR/A10_run_rosdep
echo "HOOKDIR=\$PBUILD_DIR" > \$HOME/.pbuilderrc

# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS}

# Set proper package names
PKG_NAME=ros-${ROS_DISTRO}-${PACKAGE}_${VERSION}-${RELEASE_VERSION}${DISTRO}_${ARCH}.deb

mkdir -p $WORKSPACE/pkgs
rm -fr $WORKSPACE/pkgs/*

PKGS=\`find \$HOME/pbuilder/*_result* -name *.deb || true\`

FOUND_PKG=0
for pkg in \${PKGS}; do
    echo "found \$pkg"
    #if [ $PACKAGE != 'gazebo-ros-pkgs' ]; then
    #  # Check for correctly generated packages size > 3Kb
    #  test -z \$(find \$pkg -size +3k) && echo "WARNING: empty package?" && exit 1
    #fi
    cp \${pkg} $WORKSPACE/pkgs
    FOUND_PKG=1
done
# check at least one upload
test \$FOUND_PKG -eq 1 || exit 1
DELIM

#
# Make project-specific changes here
###################################################

sudo mkdir -p /var/packages/gazebo/ubuntu
sudo pbuilder  --execute \
    --bindmounts "$WORKSPACE /var/packages/gazebo/ubuntu" \
    --basetgz $basetgz \
    -- build.sh
