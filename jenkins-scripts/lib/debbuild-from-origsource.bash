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

if [ -z $UPLOAD_SOURCEDEB ]; then
    UPLOAD_SOURCEDEB=false
fi

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

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
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools mercurial debhelper wget bash-completion

# Also get gazebo repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main" > /etc/apt/sources.list.d/gazebo.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt-get update

# Hack to avoid problem with non updated 
if [ $DISTRO = 'precise' ]; then
  echo "Skipping pbuilder check for outdated info"
  sed -i -e 's:UbuntuDistroInfo().devel():self.target_distro:g' /usr/bin/pbuilder-dist
fi

pbuilder-dist $DISTRO $ARCH create --othermirror "deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main" --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg

# Some metadata
export DEBEMAIL="osrf@osrfoundation.org"
export DEBFULLNAME="OSRF Building Farm"

# Download code
cd ${WORKSPACE}
rm -fR *.tar.* source/
wget ${SOURCE_ORIG_TARGZ}
mkdir source
tar xzf *orig* -C source
mv *orig* source/
cd source/*/
# Usally .tar.xz
wget ${DEBIAN_TARGZ}
tar xf *debian\.*
cd debian

# Need to update changelog
changelog_distro=\$(dpkg-parsechangelog -lchangelog  | grep Distribution | awk '{print \$2}')
sed -i -e "1 s:\$changelog_distro:$DISTRO:" changelog

cd ..

# Step 5: use debuild to create source package
#TODO: create non-passphrase-protected keys and remove the -uc and -us args to debuild
debuild --no-tgz-check -S -uc -us --source-option=--include-binaries -j${MAKE_JOBS}

export DEB_BUILD_OPTIONS=parallel=${MAKE_JOBS}

mkdir -p $WORKSPACE/pkgs
rm -fr $WORKSPACE/pkgs/*

if [ $UPLOAD_SOURCEDEB ]; then
    cp ../*.dsc $WORKSPACE/pkgs
    cp ../*.orig.* $WORKSPACE/pkgs
    cp ../*.debian.* $WORKSPACE/pkgs
fi

# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS}

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
