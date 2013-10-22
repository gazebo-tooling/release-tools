#!/bin/bash -x


. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

VERSION=2.81
PACKAGE=bullet

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
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools debhelper wget subversion cdbs

# Hack to avoid problem with non updated 
if [ $DISTRO = 'precise' ]; then
  echo "Skipping pbuilder check for outdated info"
  sed -i -e 's:UbuntuDistroInfo().devel():self.target_distro:g' /usr/bin/pbuilder-dist
fi

# Step 0: create/update distro-specific pbuilder environment
pbuilder-dist $DISTRO $ARCH create /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg

# Step 0: Clean up
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build

# Clean from workspace all package related files
rm -fr $WORKSPACE/"$PACKAGE"_*

# Step 1: Get the source (nightly builds or tarball)
rm -fr $WORKSPACE/bullet
svn checkout http://bullet.googlecode.com/svn/trunk/ $WORKSPACE/bullet 
# Fix compile bug in bullet
sed -i -e 's/btMultiBodyJointLimitConstraint:://g' $WORKSPACE/bullet/src/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h
cd $WORKSPACE/bullet
# need to use the debian from previous releases
wget https://launchpad.net/~efl/+archive/trunk/+files/bullet_2.81-1ppa1%7Equantal.debian.tar.gz -O debian.tar.gz
tar xfz debian.tar.gz
# get svn revision to use in package version
SVN_REV=\$(svn info | sed -n -e '/^Revision: \([0-9]*\).*$/s//\1/p')
SVN_REV=rev\$SVN_REV
# Use current distro
sed -i -e 's:quantal:$DISTRO:g' debian/changelog
sed -i -e "s:1ppa1:$RELEASE_VERSION\$SVN_REV:g" debian/changelog

# Step 5: use debuild to create source package
echo | dh_make -s --createorig -p ${PACKAGE}_${VERSION} || true

debuild -S -uc -us --source-option=--include-binaries -j${MAKE_JOBS}

export DEB_BUILD_OPTIONS="parallel=$MAKE_JOBS"
# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS}

mkdir -p $WORKSPACE/pkgs
rm -fr $WORKSPACE/pkgs/*

PKGS=\`find /var/lib/jenkins/pbuilder -name *.deb || true\`

FOUND_PKG=0
for pkg in \${PKGS}; do
    echo "found \$pkg"
    # Check for correctly generated packages size > 3Kb
    test -z \$(find \$pkg -size +3k) && exit 1
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
