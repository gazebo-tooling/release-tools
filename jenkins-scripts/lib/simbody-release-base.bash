#!/bin/bash -x


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
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools debhelper wget git dpkg-dev

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
rm -fr $WORKSPACE/simbody
git clone https://github.com/simbody/simbody.git $WORKSPACE/simbody 
cd $WORKSPACE/simbody
# TODO: REMOVE this, it is only for 3.5.1
cp doc/debian/changelog .
# Keep this line
git checkout Simbody-${VERSION}
# TODO: REMOVE this, it is only for 3.5.1
mv changelog doc/debian/

# Debian directory is in doc/
mv doc/debian .

# Use current distro
sed -i -e 's:trusty:$DISTRO:g' debian/changelog
# Use current release version
sed -i -e 's:-1~:-$RELEASE_VERSION~:' debian/changelog
# Bug in saucy doxygen makes the job hangs
if [ $DISTRO = 'saucy' ]; then
    sed -i -e '/.*dh_auto_build.*/d' debian/rules
fi

# Need to set cpp11 off for precise
if [ $DISTRO = 'precise' ]; then
  DEB_HOST_MULTIARCH=\$(dpkg-architecture -qDEB_HOST_MULTIARCH)
  sed -i -e 's#-DMAKE_BUILD_TYPE:STRING=RelWithDebInfo#-DMAKE_BUILD_TYPE:STRING=RelWithDebInfo\ -DSIMBODY_STANDARD_11=OFF\ -DCMAKE_INSTALL_LIBDIR:PATH=lib/\${DEB_HOST_MULTIARCH}#' debian/rules
fi

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
