#!/bin/bash -x

NIGHTLY_MODE=${NIGHTLY_MODE:-false}
if [ "${UPLOAD_TO_REPO}" = "nightly" ]; then
   OSRF_REPOS_TO_USE="stable nightly"
   NIGHTLY_MODE=true
fi

# Option to use $WORKSPACE/repo as container (git or hg) for the nightly source
[[ -z ${USE_REPO_DIRECTORY_FOR_NIGHTLY} ]] && USE_REPO_DIRECTORY_FOR_NIGHTLY=false

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
  if ${USE_REPO_DIRECTORY_FOR_NIGHTLY}; then
    mv ${WORKSPACE}/repo \$REAL_PACKAGE_NAME
  else
    hg clone https://bitbucket.org/${BITBUCKET_REPO}/\$REAL_PACKAGE_NAME -r default
  fi
  PACKAGE_SRC_BUILD_DIR=\$REAL_PACKAGE_NAME
  cd \$REAL_PACKAGE_NAME
  TIMESTAMP=\$(date '+%Y%m%d')
  # Store revision for use in version
  if [[ -d .hg ]]; then
    REV=\$(hg parents --template="{node|short}\n")
    TIMESTAMP="hg\$TIMESTAMP"
  elif [[ -n "$GIT_COMMIT" ]]; then
    REV=$GIT_COMMIT
    TIMESTAMP="git\$TIMESTAMP"
  elif [[ -d .git ]]; then
    REV=\$(git rev-parse HEAD)
    TIMESTAMP="git\$TIMESTAMP"
  else
    REV=0
  fi
else
  wget --quiet -O $PACKAGE_ALIAS\_$VERSION.orig.tar.bz2 $SOURCE_TARBALL_URI
  rm -rf \$REAL_PACKAGE_NAME\-$VERSION
  tar xf $PACKAGE_ALIAS\_$VERSION.orig.tar.bz2
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
    UPSTREAM_VERSION=\$( sed -n '/(/,/)/ s/.*(\([^)]*\)).*/\1 /p' ${DISTRO}/debian/changelog | head -n 1 | tr -d ' ' | sed 's:-[0-9]*~.*::' )
fi

hg up $RELEASE_REPO_BRANCH

# Should the case of new distros supported like debian
PACKAGE_RELEASE_DIR="/tmp/${PACKAGE}-release/${LINUX_DISTRO}/${DISTRO}/"
# Ubuntu special case from legacy reasons
if [[ ! -d \${PACKAGE_RELEASE_DIR} ]]; then
  PACKAGE_RELEASE_DIR="/tmp/${PACKAGE}-release/${DISTRO}"
fi

# Handle build metadata
if [ ! -f build.metadata.bash ]; then
    BUILD_METHOD="LEGACY"
else
    source build.metadata.bash
fi

case \${BUILD_METHOD} in
    "OVERWRITE_BASE")
	# 1. Clone the base branch
        hg clone https://bitbucket.org/${BITBUCKET_REPO}/$PACKAGE-release \\
	    -b \${RELEASE_BASE_BRANCH} \\
	    /tmp/base_$PACKAGE-release
	# 2. Overwrite the information
	if [[ -d ${DISTRO} ]]; then
          cp -a ${DISTRO}/debian/* /tmp/base_$PACKAGE-release/${DISTRO}/debian/
	else
	  echo "WARN: no files to overwrite where found. No ${DISTRO} directory in repo"
        fi
	# 3. Apply patches (if any)
	if [[ -d patches/ ]]; then
	    cp -a patches/*.patch /tmp/base_$PACKAGE-release
	    pushd /tmp/base_$PACKAGE-release > /dev/null
	    for p in /tmp/base_$PACKAGE-release/*.patch; do
	      patch -p1 < \$p
	    done
	    popd > /dev/null
	fi
	# 4. swap directories
	cd /tmp
	rm -fr /tmp/$PACKAGE-release
        mv /tmp/base_$PACKAGE-release /tmp/$PACKAGE-release
	PACKAGE_RELEASE_DIR="/tmp/$PACKAGE-release/${DISTRO}"
	;;
    "LEGACY")
	echo "Legacy in place. Nothing needs to be done"
	;;
esac

cd \${PACKAGE_RELEASE_DIR}

# [nightly] Adjust version in nightly mode
if $NIGHTLY_MODE; then
  NIGHTLY_VERSION_SUFFIX=\${UPSTREAM_VERSION}+\${TIMESTAMP}r\${REV}-${RELEASE_VERSION}~${DISTRO}
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
  echo | dh_make -y -s --createorig -p ${PACKAGE_ALIAS}_\${UPSTREAM_VERSION}+\${TIMESTAMP}r\${REV} > /dev/null
fi

# Adding extra directories to code. debian has no problem but some extra directories
# handled by symlinks (like cmake) in the repository can not be copied directly.
# Need special care to copy, using first a --dereference
cp -a --dereference \${PACKAGE_RELEASE_DIR}/* .
echo '# END SECTION'

echo '# BEGIN SECTION: install build dependencies'
mk-build-deps -r -i debian/control --tool 'apt-get --yes -o Debug::pkgProblemResolver=yes -o  Debug::BuildDeps=yes'
echo '# END SECTION'

if [ -f /usr/bin/rosdep ]; then
  rosdep init
fi

if $NEED_C11_COMPILER || $NEED_GCC48_COMPILER; then
echo '# BEGIN SECTION: install C++11 compiler'
if [ ${DISTRO} = 'precise' ]; then
apt-get install -y python-software-propertie software-properties-common || true
add-apt-repository ppa:ubuntu-toolchain-r/test
apt-get update
fi
apt-get install -y gcc-4.8 g++-4.8
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
g++ --version
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
    [[ \$(find \$pkg -size +3k) ]] || echo "WARNING: empty package?"
    cp \${pkg} $WORKSPACE/pkgs
    FOUND_PKG=1
done
# check at least one upload
test \$FOUND_PKG -eq 1 || exit 1
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE=${OSRF_REPOS_TO_USE:=stable}
DEPENDENCY_PKGS="devscripts \
		 ubuntu-dev-tools \
		 debhelper \
		 wget \
		 ca-certificates \
		 equivs \
		 dh-make \
		 mercurial \
		 git"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
