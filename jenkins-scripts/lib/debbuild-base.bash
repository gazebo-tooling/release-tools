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
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools mercurial debhelper wget pkg-kde-tools bash-completion

if $ENABLE_ROS; then
# get ROS repo's key, to be used in creating the pbuilder chroot (to allow it to install packages from that repo)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $DISTRO main" > /etc/apt/sources.list.d/ros-latest.list'
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
fi

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
# Fix segfault when using two repositories by setting OTHERMIRROR env variable
if $ENABLE_ROS; then
OTHERMIRROR='deb http://packages.ros.org/ros/ubuntu $DISTRO main|deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main|deb $ubuntu_repo_url $DISTRO-updates main restricted universe multiverse' pbuilder-dist $DISTRO $ARCH create --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg --mirror $ubuntu_repo_url
else
pbuilder-dist $DISTRO $ARCH create --othermirror "deb http://packages.osrfoundation.org/gazebo/ubuntu $DISTRO main|deb $ubuntu_repo_url $DISTRO-updates main restricted universe multiverse" --keyring /etc/apt/trusted.gpg --debootstrapopts --keyring=/etc/apt/trusted.gpg --mirror $ubuntu_repo_url
fi

# Step 0: Clean up
rm -rf $WORKSPACE/build
mkdir -p $WORKSPACE/build
cd $WORKSPACE/build

# Hack to support gazebo-current and friends
# REAL_PACKAGE_NAME is used to refer to code directory name
# REAL_PACKAGE_ALIAS is only affecting the name of the tarball
if [ $PACKAGE = 'gazebo-current' ] || [ $PACKAGE = 'gazebo2' ]; then
    REAL_PACKAGE_NAME='gazebo'
    REAL_PACKAGE_ALIAS='gazebo'
else
    REAL_PACKAGE_NAME=$PACKAGE
    REAL_PACKAGE_ALIAS=$PACKAGE_ALIAS
fi

# Remove number for packages like (sdformat2 or gazebo3)
REAL_PACKAGE_NAME=$(echo $PACKAGE | sed 's:[0-9]*$::g')

# Step 1: Get the source (nightly builds or tarball)
if ${NIGHTLY_MODE}; then
  hg clone https://bitbucket.org/${BITBUCKET_REPO}/\$REAL_PACKAGE_NAME -r default
  PACKAGE_SRC_BUILD_DIR=\$REAL_PACKAGE_NAME
  cd \$REAL_PACKAGE_NAME
  # Store revision for use in version
  REV=\$(hg parents --template="{node|short}\n")
else
  wget --quiet -O \$REAL_PACKAGE_ALIAS\_$VERSION.orig.tar.bz2 $SOURCE_TARBALL_URI
  rm -rf \$REAL_PACKAGE_NAME\-$VERSION
  tar xf \$REAL_PACKAGE_ALIAS\_$VERSION.orig.tar.bz2
  PACKAGE_SRC_BUILD_DIR=\$REAL_PACKAGE_NAME-$VERSION
fi

# Step 4: add debian/ subdirectory with necessary metadata files to unpacked source tarball
rm -rf /tmp/$PACKAGE-release
hg clone https://bitbucket.org/${BITBUCKET_REPO}/$PACKAGE-release /tmp/$PACKAGE-release 
cd /tmp/$PACKAGE-release
# In nightly get the default latest version from default changelog
if $NIGHTLY_MODE; then
    # TODO: remove check when multidistribution reach default branch
    if [ -f "${RELEASE_REPO_DIRECTORY}/debian/changelog" ]; then
      UPSTREAM_VERSION=\$( sed -n '/(/,/)/ s/.*(\([^)]*\)).*/\1 /p' ${RELEASE_REPO_DIRECTORY}/debian/changelog | head -n 1 | tr -d ' ' | sed 's/-.*//')
    else
      UPSTREAM_VERSION=\$( sed -n '/(/,/)/ s/.*(\([^)]*\)).*/\1 /p' ubuntu/debian/changelog | head -n 1 | tr -d ' '| sed 's/-.*//')
    fi
fi
hg up $RELEASE_REPO_BRANCH

cd /tmp/$PACKAGE-release/${RELEASE_REPO_DIRECTORY}

# Bug in saucy doxygen makes the job hangs
if [ $DISTRO = 'saucy' ]; then
    sed -i -e '/.*dh_auto_build.*/d' debian/rules
fi

# [nightly] Adjust version in nightly mode
if $NIGHTLY_MODE; then
  TIMESTAMP=\$(date '+%Y%m%d')
  RELEASE_DATE=\$(date '+%a, %d %B %Y %T -0700')
  NIGHTLY_VERSION_SUFFIX=\${UPSTREAM_VERSION}~hg\${TIMESTAMP}r\${REV}-${RELEASE_VERSION}~${DISTRO}
  # Fix the changelog
  sed -i -e "s/xxxxx/\${NIGHTLY_VERSION_SUFFIX}/g" debian/changelog
  sed -i -e "s/ddddd/\${RELEASE_DATE}/g" debian/changelog
  # TODO: Fix CMakeLists.txt ?
fi

# Get into the unpacked source directory, without explicit knowledge of that 
# directory's name
cd \`find $WORKSPACE/build -mindepth 1 -type d |head -n 1\`
# If use the quilt 3.0 format for debian (drcsim) it needs a tar.gz with sources
if $NIGHTLY_MODE; then
  rm -fr .hg*
  echo | dh_make -s --createorig -p ${PACKAGE_ALIAS}_\${UPSTREAM_VERSION}~hg\${TIMESTAMP}r\${REV} > /dev/null
fi

# Adding extra directories to code. debian has no problem but some extra directories 
# handled by symlinks (like cmake) in the repository can not be copied directly. 
# Need special care to copy, using first a --dereference
cp -a --dereference /tmp/$PACKAGE-release/${RELEASE_REPO_DIRECTORY}/* .

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

if $NEED_C11_COMPILER; then
cat > \$PBUILD_DIR/A20_install_gcc11 << DELIM_C11
#!/bin/sh
echo "Installing g++ 4.8"
apt-get install -y python-software-properties
add-apt-repository ppa:ubuntu-toolchain-r/test
apt-get update
apt-get install -y gcc-4.8 g++-4.8
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
g++ --version
DELIM_C11
chmod a+x \$PBUILD_DIR/A20_install_gcc11
fi

echo "HOOKDIR=\$PBUILD_DIR" > \$HOME/.pbuilderrc

export DEB_BUILD_OPTIONS=parallel=${MAKE_JOBS}

# Step 6: use pbuilder-dist to create binary package(s)
pbuilder-dist $DISTRO $ARCH build ../*.dsc -j${MAKE_JOBS} --mirror $ubuntu_repo_url

# Set proper package names
if $NIGHTLY_MODE; then
  PKG_NAME=${PACKAGE_ALIAS}_\${NIGHTLY_VERSION_SUFFIX}_${ARCH}.deb
  DBG_PKG_NAME=${PACKAGE_ALIAS}-dbg_\${NIGHTLY_VERSION_SUFFIX}_${ARCH}.deb
else
  PKG_NAME=${PACKAGE_ALIAS}_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
  DBG_PKG_NAME=${PACKAGE_ALIAS}-dbg_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
fi

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
