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

PACKAGE_UNDERSCORE_NAME=${PACKAGE//-/_}

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

echo '# BEGIN SECTION: import the sources'
rm -rf /tmp/$PACKAGE-release
git clone ${UPSTREAM_RELEASE_REPO} /tmp/$PACKAGE-release
cd /tmp/$PACKAGE-release

FULL_VERSION=$VERSION-$RELEASE_VERSION
FULL_DEBIAN_BRANCH_NAME=ros-$ROS_DISTRO-$PACKAGE\_\$FULL_VERSION\_$DISTRO

git checkout release/$ROS_DISTRO/$PACKAGE_UNDERSCORE_NAME/\$FULL_VERSION
git checkout debian/\$FULL_DEBIAN_BRANCH_NAME

if [ -f debian/control ]; then
cat debian/control
fi
echo '# END SECTION'

echo '# BEGIN SECTION: install build dependencies'
mk-build-deps -r -i debian/control --tool 'apt-get --yes -o Debug::pkgProblemResolver=yes -o  Debug::BuildDeps=yes'
echo '# END SECTION'

echo '# BEGIN SECTION: run rosdep'
rosdep init
echo '# END SECTION'

echo '# BEGIN SECTION: create source package'
# Handle the dh_make -y option (new in Xenial) or workaround using old method
# original idea: https://github.com/scarygliders/X11RDP-o-Matic/pull/61

dh_make -h | grep -q -- -y && \
    DH_MAKE_Y=true || DH_MAKE_Y=false

if \$DH_MAKE_Y
then
  # true is to avoid the error on already debian/ directory
  dh_make -y -s --createorig -p ros-$ROS_DISTRO-${PACKAGE}_${VERSION} || true
else
  echo | dh_make -s --createorig -p ros-$ROS_DISTRO-${PACKAGE}_${VERSION} || true
fi
echo '# END SECTION'


echo '# BEGIN SECTION: build package'
debuild --no-tgz-check -uc -us -S --source-option=--include-binaries
echo '# END SECTION'

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
USE_ROS_REPO=true
DEPENDENCY_PKGS="devscripts \
		 ubuntu-dev-tools \
                 ubuntu-keyring \
		 debhelper \
                 cdbs \
		 wget \
		 ca-certificates \
		 equivs \
		 dh-make \
		 mercurial \
		 git \
                 python-openssl \
                 ca-certificates \
 		 python-rosdep"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
