#!/bin/bash -x

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

# The git plugin leaves a repository copy with a detached HEAD
# state. gbp does not like it thus the need of using --git-ignore-branch
export GBP_COMMAND="gbp buildpackage -j${MAKE_JOBS} --git-ignore-new --git-ignore-branch -uc -us"
export REPO_PATH="$WORKSPACE/repo"

# Historically the job run git clone. New version leave it for jenkins
# but we keep backwards compatibility with the following checks:
export CLONE_NEEDED=true
if [[ -d ${REPO_PATH} ]]; then
  export CLONE_NEEDED=false
fi

# Respect BRANCH parameter if set. Disable checkout if not BRANCh is set
if [[ -z ${BRANCH} ]]; then
  export BRANCH=master
  export CLONE_NEEDED=false
fi

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
#!/usr/bin/env bash
set -ex

if ${CLONE_NEEDED}; then
echo '# BEGIN SECTION: clone the git repo'
rm -fr ${REPO_PATH}
git clone $GIT_REPOSITORY ${REPO_PATH}
cd ${REPO_PATH}
git checkout ${BRANCH}
echo '# END SECTION'
fi

cd ${REPO_PATH}

echo '# BEGIN SECTION: install build dependencies'
mk-build-deps -r -i debian/control --tool 'apt-get --yes -o Debug::pkgProblemResolver=yes -o  Debug::BuildDeps=yes'
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
  # libbullet-dev is the name in Ubuntu, libbullet2.82.dev is the one in OSRF
  sed -i -e 's:libbullet-dev:libbullet2.82-dev:g' debian/control
fi
if [ $DISTRO = 'trusty' ] || [ $DISTRO = 'utopic' ]; then
  # libsdformat-dev is the name in Ubuntu, libsdformat2-dev is the one in OSRF
  sed -i -e 's:libsdformat-dev:libsdformat2-dev:g' debian/control 
fi

# Do not perform symbol checking
rm -fr debian/*.symbols
echo '# END SECTION'

echo "# BEGIN SECTION: create source package \${OSRF_VERSION}"
rm -f ../*.orig.*
${GBP_COMMAND} -S

cp ../*.dsc $WORKSPACE/pkgs
cp ../*.tar.gz $WORKSPACE/pkgs
cp ../*.orig.* $WORKSPACE/pkgs
cp ../*.debian.* $WORKSPACE/pkgs
echo '# END SECTION'

echo '# BEGIN SECTION: create deb packages'
rm -f $WORKSPACE/pkgs/*.deb
rm -f ../*.deb
${GBP_COMMAND}
echo '# END SECTION'

echo '# BEGIN SECTION: export pkgs'
lintian -L -i ../*.changes
PKGS=\`find ../ -name *.deb || true\`

FOUND_PKG=0
for pkg in \${PKGS}; do
    echo "found \$pkg"
    cp \${pkg} $WORKSPACE/pkgs
    FOUND_PKG=1
done
# check at least one upload
test \$FOUND_PKG -eq 1 || exit 1
echo '# END SECTION'

echo '# BEGIN SECTION: clean up git build'
cd $REPO_PATH
git clean -f -d
echo '# END SECTION'
DELIM

OSRF_REPOS_TO_USE="stable"
DEPENDENCY_PKGS="devscripts \
		 ubuntu-dev-tools \
		 debhelper \
		 wget \
		 ca-certificates \
		 equivs \
		 git \
		 git-buildpackage"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
