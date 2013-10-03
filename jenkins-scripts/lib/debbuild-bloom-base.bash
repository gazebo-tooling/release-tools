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

# Install deb-building tools
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools mercurial git debhelper wget

# get ROS repo's key, to be used in creating the pbuilder chroot (to allow it to install packages from that repo)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $DISTRO main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
# Also get drc repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu $DISTRO main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | apt-key add -
apt-get update

# Hack to avoid problem with non updated 
if [ $DISTRO = 'precise' ]; then
  echo "Skipping pbuilder check for outdated info"
  sed -i -e 's:UbuntuDistroInfo().devel():self.target_distro:g' /usr/bin/pbuilder-dist
fi

# Step 0: create/update distro-specific pbuilder environment
pbuilder-dist $DISTRO $ARCH create --othermirror "deb http://packages.ros.org/ros/ubuntu $DISTRO main|deb http://packages.osrfoundation.org/drc/ubuntu $DISTRO main" --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg

# Step 0: Clean up
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build

# Step 4: checkout bloom software -release repo 
rm -rf /tmp/$PACKAGE-release
git clone ${UPSTREAM_RELEASE_REPO} /tmp/$PACKAGE-release
cd /tmp/$PACKAGE-release

FULL_VERSION=$VERSION-$RELEASE_VERSION

git checkout -b tags/release/$DISTRO/$PACKAGE/\$FULL_VERSION
git checkout tags/debian/$PACKAGE\_\$FULL_VERSION_$DISTRO

# Step 5: use debuild to create source package
#TODO: create non-passphrase-protected keys and remove the -uc and -us args to debuild
debuild --no-tgz-check -S -uc -us --source-option=--include-binaries -j${MAKE_JOBS}

PBUILD_DIR=\$HOME/.pbuilder
mkdir -p \$PBUILD_DIR
cat > \$PBUILD_DIR/A10_run_rosdep << DELIM_ROS_DEP
#!/bin/sh
if [ -f /usr/bin/rosdep ]; then
  # root share the same /tmp/buildd HOME than pbuilder user. Need to specify the root
  # HOME=/root otherwise it will make cache created during ros call forbidden to 
  # access to pbuilder user.
  HOME=/root rosdep init
fi
DELIM_ROS_DEP
chmod a+x \$PBUILD_DIR/A10_run_rosdep
echo "HOOKDIR=\$PBUILD_DIR" > \$HOME/.pbuilderrc

# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS}

# Set proper package names
PKG_NAME=${PACKAGE_NAME}_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
DBG_PKG_NAME=${PACKAGE_NAME}-dbg_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb

mkdir -p $WORKSPACE/pkgs
rm -fr $WORKSPACE/pkgs/*

# Both paths are need, beacuse i386 use a different path
MAIN_PKGS="/var/lib/jenkins/pbuilder/${DISTRO}_result/\${PKG_NAME} /var/lib/jenkins/pbuilder/${DISTRO}-${ARCH}_result/\${PKG_NAME}"
DEBUG_PKGS="/var/lib/jenkins/pbuilder/${DISTRO}_result/\${DBG_PKG_NAME} /var/lib/jenkins/pbuilder/${DISTRO}-${ARCH}_result/\${DBG_PKG_NAME}"

FOUND_PKG=0
for pkg in \${MAIN_PKGS}; do
    echo "looking for \$pkg"
    if [ -f \${pkg} ]; then
        echo "found \$pkg"
	# Check for correctly generated packages size > 3Kb
        test -z \$(find \$pkg -size +3k) && exit 1
	cp \${pkg} $WORKSPACE/pkgs
        FOUND_PKG=1
        break;
    fi
done
test \$FOUND_PKG -eq 1 || exit 1

FOUND_PKG=0
for pkg in \${DEBUG_PKGS}; do
    if [ -f \${pkg} ]; then
        # Check for correctly generated debug packages size > 3Kb
        # when not valid instructions in rules/control it generates 1.5K package
        test -z \$(find \$pkg -size +3k) && exit 1
        cp \${pkg} $WORKSPACE/pkgs
        FOUND_PKG=1
        break;
    fi
done
test \$FOUND_PKG -eq 1 || echo "No debug packages found. No upload"
DELIM

#
# Make project-specific changes here
###################################################

sudo mkdir -p /var/packages/gazebo/ubuntu
sudo pbuilder  --execute \
    --bindmounts "$WORKSPACE /var/packages/gazebo/ubuntu" \
    --basetgz $basetgz \
    -- build.sh
