#!/bin/bash -x

NIGHTLY_MODE=${NIGHTLY_MODE:-false}
if [ "${VERSION}" = "nightly" ]; then
   OSRF_REPOS_TO_USE="${OSRF_REPOS_TO_USE:-stable nightly}"
   NIGHTLY_MODE=true
   # SOURCE_TARBALL_URI is reused in nightly mode to indicate the branch
   # to built nightly packages from
   NIGHTLY_SRC_BRANCH=${SOURCE_TARBALL_URI}
   # There are many problem in the nightlies with package versions preventing
   # the dependency solver to work properly. Set INVALIDATE_DOCKER_CACHE
   export INVALIDATE_DOCKER_CACHE=true
fi

# Option to use $WORKSPACE/repo as container (git or hg) for the nightly source
[[ -z ${USE_REPO_DIRECTORY_FOR_NIGHTLY} ]] && USE_REPO_DIRECTORY_FOR_NIGHTLY=false

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
. ${SCRIPT_DIR}/lib/_gazebo_utils.sh

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
    git clone https://github.com/gazebosim/\$REAL_PACKAGE_NAME -b ${NIGHTLY_SRC_BRANCH}
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
    TIMESTAMP="nightly+git\$TIMESTAMP"
  else
    REV=0
  fi
else
  # Some combinations does not known about AWS certificate from S3
  if [[ ${LINUX_DISTRO} == debian ]] || [[ ${DISTRO} == 'focal' && ${ARCH} == 'armhf' ]]; then
     no_check_cert_str='--no-check-certificate'
  fi
  wget \$no_check_cert_str --quiet -O orig_tarball $SOURCE_TARBALL_URI || \
    echo rerunning wget without --quiet since it failed && \
    wget \$no_check_cert_str -O orig_tarball $SOURCE_TARBALL_URI
  TARBALL_EXT=${SOURCE_TARBALL_URI/*tar./}
  mv orig_tarball $PACKAGE_ALIAS\_$VERSION.orig.tar.\${TARBALL_EXT}
  rm -rf \$REAL_PACKAGE_NAME\-$VERSION
  tar xf $PACKAGE_ALIAS\_$VERSION.orig.tar.*
  PACKAGE_SRC_BUILD_DIR=\$REAL_PACKAGE_NAME-$VERSION
fi

# Step 4: add debian/ subdirectory with necessary metadata files to unpacked source tarball
rm -rf /tmp/$PACKAGE-release
git clone https://github.com/gazebo-release/$PACKAGE-release -b $RELEASE_REPO_BRANCH /tmp/$PACKAGE-release
cd /tmp/$PACKAGE-release
# In nightly get the default latest version from default changelog
if $NIGHTLY_MODE; then
    # TODO: migrate to dpkg-parsechangelog
    # dpkg-parsechangelog| grep Version | cut -f2 -d' '
    UPSTREAM_VERSION=\$( sed -n '/(/,/)/ s/.*(\([^)]*\)).*/\1 /p' ${DISTRO}/debian/changelog | head -n 1 | tr -d ' ' | sed 's:-[0-9]*~.*::' )
fi

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
        git clone https://github.com/gazebo-release/\${PACKAGE}-release \\
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

# Helper for transition of ign to gz
PACKAGE_ALIAS=${PACKAGE_ALIAS}
SRC_PACKAGE_NAME=\$(grep-dctrl -sSource -n  '' debian/control)
if [[ \${SRC_PACKAGE_NAME} != \${SRC_PACKAGE_NAME/gz-} ]]; then
  PACKAGE_ALIAS=\${SRC_PACKAGE_NAME}
fi

# [nightly] Adjust version in nightly mode
if $NIGHTLY_MODE; then
  NIGHTLY_VERSION_SUFFIX=\${UPSTREAM_VERSION}+\${TIMESTAMP}+${RELEASE_VERSION}r\${REV}-${RELEASE_VERSION}~${DISTRO}
  debchange --package \${PACKAGE_ALIAS} \\
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
  rm -fr .hg* .git*
  # Versions of dh-make lower than 2.202003 are buggy using defaultless
  # see: https://salsa.debian.org/debian/dh-make/-/merge_requests/8
  extra_dh_make_str='--defaultless'
  if dpkg --compare-versions \$(apt-cache show dh-make | sed -n "s/Version: \\(.*\\)/\\1/p") lt 2.202003; then
    extra_dh_make_str=''
  fi
  echo | dh_make -y -s --createorig \${extra_dh_make_str} -p\${PACKAGE_ALIAS}_\${UPSTREAM_VERSION}+\${TIMESTAMP}+${RELEASE_VERSION}r\${REV} > /dev/null
  rm -fr debian/
fi

# Adding extra directories to code. debian has no problem but some extra directories
# handled by symlinks (like cmake) in the repository can not be copied directly.
# Need special care to copy, using first a --dereference
cp -a --dereference \${PACKAGE_RELEASE_DIR}/* .
echo '# END SECTION'

echo '# BEGIN SECTION: install build dependencies'
sudo apt-get update
# buster workaround for dwz. backports repository does not have priority over main
# explicit the version wanted
if [ ${DISTRO} = 'buster' ]; then
  sudo apt-get install -y dwz=0.13-5~bpo10+1
fi

timeout=0
# Help to debug race conditions in nightly generation or other problems with versions
if ${NIGHTLY_MODE}; then
  apt-cache show *gz-* | ( grep 'Package\\|Version' || true)
  # 5 minutes to give time to the uploader
  timeout=300
fi

${MKBUILD_INSTALL_DEPS}

if [ -f /usr/bin/rosdep ]; then
  rosdep init
fi

# Be sure that a previous bug using g++8 compiler is not present anymore
if [[ ${DISTRO} == 'jammy' || ${DISTRO} == 'focal' ]]; then
 [[ \$(/usr/bin/gcc --version | grep 'gcc-8') ]] && ( echo "gcc-8 version found. A bug." ; exit 1 )
elif $INSTALL_C17_COMPILER; then
  echo '# BEGIN SECTION: install C++17 compiler'
  sudo apt-get install -y gcc-8 g++-8
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
  g++ --version
  echo '# END SECTION'
fi

echo '# BEGIN SECTION: create source package' \${OSRF_VERSION}

# lintian triggers a problem on arm in Focal when using qemu, avoid it
no_lintian_param=""
if [[ ${DISTRO} == 'focal' && (${ARCH} == 'arm64' || ${ARCH} == 'armhf') ]]; then
  no_lintian_param="--no-lintian"
fi

# our packages.o.o running xenial does not support default zstd compression of
# .deb files in jammy. Keep using xz. Not a trivial change, requires wrapper over dpkg-deb
if [[ ${DISTRO} == 'jammy' ]]; then
  sudo bash -c 'echo \#\!/bin/bash > /usr/local/bin/dpkg-deb'
  sudo bash -c 'echo "/usr/bin/dpkg-deb -Zxz \\\$@" >> /usr/local/bin/dpkg-deb'
  sudo cat /usr/local/bin/dpkg-deb
  sudo chmod +x /usr/local/bin/dpkg-deb
  export PATH=/usr/local/bin:\$PATH
  preserve_path='--preserve-envvar PATH'
fi

debuild \${no_lintian_param} \${preserve_path} --no-tgz-check -uc -us -S --source-option=--include-binaries

cp ../*.dsc $WORKSPACE/pkgs
cp ../*.orig.* $WORKSPACE/pkgs
cp ../*.tar.* $WORKSPACE/pkgs
# debian is only generated in quilt format, native does not have it
cp ../*.debian.* $WORKSPACE/pkgs || true
echo '# END SECTION'

# Enable compat level 12 to get --list-missing enabled by default, if the
# support is found in debhelper
if [[ -n  \$(grep -R 'compat 12' /usr/share/perl5/Debian/Debhelper/Dh_*.pm) ]]; then
  echo 12 > debian/compat
fi

# Hack to avoid problems with dwz symbols starting in Ubuntu Disco if tmp and debugtmp are the
# same the build fails copying files because they are the same
if [[ $DISTRO != 'bionic' ]]; then
  sudo sed -i -e 's:dwz" and:dwz" and (\$tmp ne \$debugtmp) and:' /usr/bin/dh_strip
fi

echo '# BEGIN SECTION: create deb packages'
debuild \${no_lintian_param} \${preserve_path} --no-tgz-check -uc -us --source-option=--include-binaries -j${MAKE_JOBS}
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

cat /etc/apt/sources.list
# Run only autopkgtest on amd64
# see https://github.com/gazebosim/gz-common/issues/484
if [[ ${ARCH} == 'amd64' ]]; then
  ${DEBBUILD_AUTOPKGTEST}
fi
DELIM

OSRF_REPOS_TO_USE=${OSRF_REPOS_TO_USE:=stable}
DEPENDENCY_PKGS="devscripts \
                ubuntu-dev-tools \
                debhelper \
                wget \
                ca-certificates \
                equivs \
                dh-make \
                git \
                autopkgtest"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
