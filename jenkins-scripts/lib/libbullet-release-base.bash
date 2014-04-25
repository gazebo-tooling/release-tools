#!/bin/bash -x

# Do not use ROS
export ENABLE_ROS=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

UPSTREAM_VERSION="2.82-r2704"
VERSION="2.82"
PACKAGE=bullet

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

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
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools debhelper wget subversion cdbs mercurial ca-certificates

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

# Step 1: Get the source from 
rm -fr $WORKSPACE/bullet
cd $WORKSPACE
wget https://bullet.googlecode.com/files/bullet-$UPSTREAM_VERSION.tgz -O bullet.tgz
tar xzf bullet.tgz
mv bullet-$UPSTREAM_VERSION bullet

cd $WORKSPACE/bullet

# Debian information
rm -fr $WORKSPACE/debian
mkdir $WORKSPACE/debian
hg clone https://bitbucket.org/osrf/bullet-2.82-debian-release $WORKSPACE/debian
cp -a --dereference $WORKSPACE/debian/old/debian $WORKSPACE/bullet

cd $WORKSPACE/bullet
# Use current distro
sed -i -e 's:precise:$DISTRO:g' debian/changelog
sed -i -e "s:osrf1:osrf$RELEASE_VERSION:g" debian/changelog

# Step 5: use debuild to create source package
echo | dh_make -s --createorig -p ${PACKAGE}_${VERSION} || true

debuild -S -uc -us --source-option=--include-binaries -j${MAKE_JOBS}

export DEB_BUILD_OPTIONS="parallel=$MAKE_JOBS"

# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS}

mkdir -p $WORKSPACE/pkgs
rm -fr $WORKSPACE/pkgs/*

PKGS=\`find /var/lib/jenkins/pbuilder/*_result* -name *.deb || true\`

FOUND_PKG=0
for pkg in \${PKGS}; do
    echo "found \$pkg"
    # Check for correctly generated packages size > 3Kb
    test -z \$(find \$pkg -size +3k) && echo "WARNING: empty package?" 
    # && exit 1
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
