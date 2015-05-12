#!/bin/bash -x

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# If no value for MULTIARCH_SUPPORT was submitted and
# distro is precise, disable the multiarch, this is generally
# since the use of GNUINSTALLDIRs
if [[ -z ${MULTIARCH_SUPPORT} ]]; then
  if [[ $DISTRO == 'precise' ]]; then
    MULTIARCH_SUPPORT=false
  fi
fi

# Use defaul branch if not sending BRANCH parameter
[[ -z ${BRANCH} ]] && export BRANCH=default

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
# equivcs for mk-build-depends
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools debhelper wget cdbs ca-certificates dh-autoreconf autoconf equivs git

# Also get gazebo repo's key, to be used in getting Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main" > /etc/apt/sources.list.d/gazebo.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt-get update

# Hack to avoid problem with non updated
if [ $DISTRO = 'precise' ]; then
  echo "Skipping pbuilder check for outdated info"
  sed -i -e 's:UbuntuDistroInfo().devel():self.target_distro:g' /usr/bin/pbuilder-dist
fi

# Step 0: create/update distro-specific pbuilder environment
pbuilder-dist $DISTRO $ARCH create --othermirror "deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main|deb $ubuntu_repo_url $DISTRO-updates main restricted universe multiverse" --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg --mirror $ubuntu_repo_url

# Step 0: Clean up
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build

# Clean from workspace all package related files
rm -fr $WORKSPACE/"$PACKAGE"_*

echo '# BEGIN SECTION: clone the git repo'
rm -fr $WORKSPACE/repo
git clone $GIT_REPOSITORY $WORKSPACE/repo
cd $WORKSPACE/repo
git checkout -b ${BRANCH}
echo '# END SECTION'

echo '# BEGIN SECTION: install build dependencies'
mk-build-deps -i debian/control --tool 'apt-get --no-install-recommends --yes'
rm *build-deps*.deb
echo '# END SECTION'

echo '# BEGIN SECTION: build version and distribution'
VERSION=\$(dpkg-parsechangelog  | grep Version | awk '{print \$2}')
VERSION_NO_REVISION=\$(echo \$VERSION | sed 's:-.*::')
OSRF_VERSION=\$VERSION\osrf${RELEASE_VERSION}~${DISTRO}${RELEASE_ARCH_VERSION}
sed -i -e "s:\$VERSION:\$OSRF_VERSION:g" debian/changelog

# Use current distro (unstable or experimental are in debian)
changelog_distro=\$(dpkg-parsechangelog | grep Distribution | awk '{print \$2}')
sed -i -e "1 s:\$changelog_distro:$DISTRO:" debian/changelog

# When backported from Vivid (or above) to Trusty/Utopic some packages are not
# avilable or names are different
if [ $DISTRO = 'trusty' ]; then
  # libbullet-dev is only 2.81 in trusty, don't build against it
  sed -i '/bullet/d' debian/control
  # use libogre1.8-dev in trusty per https://github.com/ros-infrastructure/rep/pull/89#issuecomment-69232117
  sed -i -e 's:libogre-1\.9-dev:libogre-1.8-dev:g' debian/control
fi
if [ $DISTRO = 'trusty' ] || [ $DISTRO = 'utopic' ]; then
  # libsdformat-dev is the name in Ubuntu, libsdformat2-dev is the one in OSRF
  sed -i -e 's:libsdformat-dev:libsdformat2-dev:g' debian/control
fi

# In precise, no multiarch paths was implemented in GNUInstallDirs. Remove it.
if ! $MULTIARCH_SUPPORT; then
  sed -i -e 's:/\*/:/:g' debian/*.install
fi

# Do not perform symbol checking
rm -fr debian/*.symbols
echo '# END SECTION'

echo '# BEGIN SECTION: create source package ${OSRF_VERSION}'
PACKAGE=\$(dpkg-parsechangelog --show-field Source)
# Step 5: use debuild to create source package
echo | dh_make -s --createorig -p \${PACKAGE}_\${VERSION_NO_REVISION} || true
# Cleanup previous dsc files
rm -f ../*.dsc

debuild -S -uc -us --source-option=--include-binaries -j${MAKE_JOBS}

mkdir -p $WORKSPACE/pkgs
rm -fr $WORKSPACE/pkgs/*

cp ../*.dsc $WORKSPACE/pkgs
cp ../*.orig.* $WORKSPACE/pkgs
cp ../*.debian.* $WORKSPACE/pkgs
echo '# END SECTION'

echo '# BEGIN SECTION: create deb packages'
export DEB_BUILD_OPTIONS="parallel=$MAKE_JOBS"
# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS} --mirror $ubuntu_repo_url
echo '# END SECTION'

echo '# BEGIN SECTION: export pkgs'
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
echo '# END SECTION'

# clean up disk space
rm -fr ${WORKSPACE}/build
DELIM

#
# Make project-specific changes here
###################################################

sudo mkdir -p /var/packages/gazebo/ubuntu
sudo pbuilder  --execute \
    --bindmounts "$WORKSPACE /var/packages/gazebo/ubuntu" \
    --basetgz $basetgz \
    -- build.sh
