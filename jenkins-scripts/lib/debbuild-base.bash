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
set -ex

# Install deb-building tools
apt-get install -y pbuilder fakeroot debootstrap devscripts dh-make ubuntu-dev-tools mercurial debhelper reprepro wget

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

# Step 1: Get the source (nightly builds or tarball)
if ${NIGHTLY_MODE}; then
  apt-get install -y mercurial
  hg clone https://bitbucket.org/osrf/$PACKAGE -r default
  PACKAGE_SRC_BUILD_DIR=$PACKAGE
  cd $PACKAGE
  # Store revision for use in version
  REV=\$(hg parents --template="{node|short}\n")
else
  wget --quiet -O ${PACKAGE_ALIAS}_$VERSION.orig.tar.bz2 $SOURCE_TARBALL_URI
  rm -rf $PACKAGE-$VERSION
  tar xf ${PACKAGE_ALIAS}_$VERSION.orig.tar.bz2
  PACKAGE_SRC_BUILD_DIR=$PACKAGE-$VERSION
  # Hack to support sdf special name for bitbucket
  if [ '$PACKAGE' = 'sdf' ]; then
      PACKAGE_SRC_BUILD_DIR="sdformat-$VERSION"
  fi
fi

# Step 4: add debian/ subdirectory with necessary metadata files to unpacked source tarball
rm -rf /tmp/$PACKAGE-release
hg clone https://bitbucket.org/osrf/$PACKAGE-release /tmp/$PACKAGE-release 
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

cd $WORKSPACE/build/\$PACKAGE_SRC_BUILD_DIR
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
debuild --no-tgz-check -S -uc -us --source-option=--include-binaries

# Step 5.1: define a pbuilder hack to include all things needed to run from inside
# pbuilder.
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
pbuilder-dist $DISTRO $ARCH build ../*.dsc

# Step 7: upload resulting .deb
sudo apt-get install -y openssh-client
cd /var/packages/gazebo/ubuntu

# Set proper package names
if $NIGHTLY_MODE; then
  PKG_NAME=${PACKAGE_ALIAS}_\${NIGHTLY_VERSION_SUFFIX}_${ARCH}.deb
  DBG_PKG_NAME=${PACKAGE_ALIAS}-dbg_\${NIGHTLY_VERSION_SUFFIX}_${ARCH}.deb
else
  PKG_NAME=${PACKAGE_ALIAS}_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
  DBG_PKG_NAME=${PACKAGE_ALIAS}-dbg_${VERSION}-${RELEASE_VERSION}~${DISTRO}_${ARCH}.deb
fi

MAIN_PKGS="/var/lib/jenkins/pbuilder/${DISTRO}-${ARCH}_result/\${PKG_NAME} /var/lib/jenkins/pbuilder/${DISTRO}_result/\${PKG_NAME}"
DEBUG_PKGS="/var/lib/jenkins/pbuilder/${DISTRO}-${ARCH}_result/\${DBG_PKG_NAME} /var/lib/jenkins/pbuilder/${DISTRO}_result/\${DBG_PKG_NAME}"

FOUND_PKG=0
for pkg in \${MAIN_PKGS}; do
    echo "looking for \$pkg"
    if [ -f \${pkg} ]; then
        echo "found \$pkg"
	# Check for correctly generated packages size > 3Kb
        test -z \$(find \$pkg -size +3k) && exit 1
        GNUPGHOME=$WORKSPACE/gnupg reprepro includedeb $DISTRO \${pkg}
        scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa \${pkg} ubuntu@gazebosim.org:/var/www/assets/distributions
        if $NIGHTLY_MODE; then
          # Be sure we are not removing something not -nightly
          if [ `echo $PACKAGE_ALIAS | sed -e 's:nightly::'` == $PACKAGE_ALIAS ]; then
               echo "Sanity check fail! Close to remove something with no nightly in the name" && exit 1
	  fi
          # Remove all nightly version except latest three
          ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@gazebosim.org "ls -t /var/www/assets/distributions/${PACKAGE_ALIAS}_*~${DISTRO}_${ARCH}.deb | sed -e '1,3d' | xargs -d '\n' rm -f"
        fi
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
        # test -z \$(find \$pkg -size +1.5k) && exit 1
        GNUPGHOME=$WORKSPACE/gnupg reprepro includedeb $DISTRO \${pkg}
        scp -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa \${pkg} ubuntu@gazebosim.org:/var/www/assets/distributions
        if $NIGHTLY_MODE; then
          # Be sure we are not removing something not -nightly
          if [ `echo $PACKAGE_ALIAS | sed -e 's:nightly::'` == $PACKAGE_ALIAS ]; then
               echo "Sanity check fail! Close to remove something with no nightly in the name" && exit 1
	  fi
          # Remove all nightly version except latest three
          ssh -o StrictHostKeyChecking=no -i $WORKSPACE/id_rsa ubuntu@gazebosim.org "ls -t /var/www/assets/distributions/${PACKAGE_ALIAS}-dbg_*~${DISTRO}_${ARCH}.deb | sed -e '1,3d' | xargs -d '\n' rm -f"
        fi
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

sudo pbuilder  --execute \
    --bindmounts "$WORKSPACE /var/packages/gazebo/ubuntu" \
    --basetgz $basetgz \
    -- build.sh
