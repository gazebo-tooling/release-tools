#!/bin/bash -x

NIGHTLY_MODE=false
if [ "${UPLOAD_TO_REPO}" = "nightly" ]; then
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

cd $WORKSPACE/build

export DEBFULLNAME="OSRF Jenkins"
export DEBEMAIL="build@osrfoundation.org"

echo '# BEGIN SECTION: import the debian metadata'

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
    # TODO: migrate to dpkg-parsechangelog
    # dpkg-parsechangelog| grep Version | cut -f2 -d' '
    UPSTREAM_VERSION=\$( sed -n '/(/,/)/ s/.*(\([^)]*\)).*/\1 /p' ${DISTRO}/debian/changelog | head -n 1 | tr -d ' ' | sed 's:~.*::')
fi
hg up $RELEASE_REPO_BRANCH

cd /tmp/$PACKAGE-release/${DISTRO}

# [nightly] Adjust version in nightly mode
if $NIGHTLY_MODE; then
  TIMESTAMP=\$(date '+%Y%m%d')
  NIGHTLY_VERSION_SUFFIX=\${UPSTREAM_VERSION}~hg\${TIMESTAMP}r\${REV}-${RELEASE_VERSION}~${DISTRO}
  # Update the changelog
  debchange --package ${PACKAGE} \\
              --newversion \${NIGHTLY_VERSION_SUFFIX} \\
              --distribution ${DISTRO} \\
              --force-distribution \\
              --changelog=debian/changelog -- "Nightly release: \${NIGHTLY_VERSION_SUFFIX}"
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
cp -a --dereference /tmp/$PACKAGE-release/${DISTRO}/* .
echo '# END SECTION'

echo '# BEGIN SECTION: install build dependencies'
mk-build-deps -r -i debian/control --tool 'apt-get --no-install-recommends --yes'
echo '# END SECTION'

if [ -f /usr/bin/rosdep ]; then
  rosdep init
fi

if $NEED_C11_COMPILER || $NEED_GCC48_COMPILER; then
echo '# BEGIN SECTION: install C++11 compiler'
apt-get install -y python-software-properties
add-apt-repository ppa:ubuntu-toolchain-r/test
apt-get update
apt-get install -y gcc-4.8 g++-4.8
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
g++ --version
chmod a+x \$PBUILD_DIR/A20_install_gcc11
echo '# END SECTION'
fi

echo '# BEGIN SECTION: create source package' \${OSRF_VERSION}
debuild --no-tgz-check -uc -us -S --source-option=--include-binaries

cp ../*.dsc $WORKSPACE/pkgs
cp ../*.orig.* $WORKSPACE/pkgs
cp ../*.tar.* $WORKSPACE/pkgs
# debian is only generated in quilt format, native does not have it
cp ../*.debian.* $WORKSPACE/pkgs || true
echo '# END SECTION'

echo '# BEGIN SECTION: create deb packages'
debuild --no-tgz-check -uc -us --source-option=--include-binaries -j${MAKE_JOBS}
echo '# END SECTION'

echo '# BEGIN SECTION: export pkgs'
PKGS=\`find .. -name '*.deb' || true\`

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
echo '# END SECTION'
DELIM

USE_OSRF_REPO=true
DEPENDENCY_PKGS="devscripts \
		 ubuntu-dev-tools \
		 debhelper \
		 wget \
		 ca-certificates \
		 equivs \
		 dh-make \
		 mercurial"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
