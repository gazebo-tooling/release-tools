#!/bin/bash -x

# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh
. ${SCRIPT_DIR}/lib/_gazebo_utils.sh

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
echo '# END SECTION'
fi

cd ${REPO_PATH}
git checkout upstream
git checkout ${BRANCH}
git pull
git branch

if [ `expr length "${DEBIAN_GIT_PREINSTALL_HOOK} "` -gt 1 ]; then
echo '# BEGIN SECTION: running pre install hook'
${DEBIAN_GIT_PREINSTALL_HOOK}
echo '# END SECTION'
fi

if [[ ${DISTRO} == 'focal' && ${ARCH} == 'arm64' ]]; then
    # Did not find the way of avoid lintian in gbp call
    sudo ln -sf /bin/true /usr/bin/lintian
fi

# our packages.o.o running xenial does not support default zstd compression of
# .deb files in jammy. Keep using xz. Not a trivial change, requires wrapper over dpkg-deb
if [[ ${DISTRO} == 'jammy' ]]; then
  sudo bash -c 'echo \#\!/bin/bash > /usr/local/bin/dpkg-deb'
  sudo bash -c 'echo "/usr/bin/dpkg-deb -Zxz \\\$@" >> /usr/local/bin/dpkg-deb'
  sudo cat /usr/local/bin/dpkg-deb
  sudo chmod +x /usr/local/bin/dpkg-deb
  export PATH=/usr/local/bin:\$PATH
fi

${MKBUILD_INSTALL_DEPS}

VERSION=\$(dpkg-parsechangelog  | grep Version | awk '{print \$2}')
VERSION_NO_REVISION=\$(echo \$VERSION | sed 's:-.*::')
OSRF_VERSION=\$VERSION\osrf${RELEASE_VERSION}~${DISTRO}${RELEASE_ARCH_VERSION}

echo "# BEGIN SECTION: check that pristine-tar is updated"
git checkout -f pristine-tar || { echo "W: probably miss the pristine-tar branch" && exit 1; }
# The tilde (~) is not allow in git tag and changed to underscore (_)
PRISTINE_VERSION_NO_REVISION=\$(echo \${VERSION_NO_REVISION} | sed 's:~:_:g')
if [[ -z \$(git tag | grep upstream/\${PRISTINE_VERSION_NO_REVISION}) ]]; then
   echo "W: \${PRISTINE_VERSION_NO_REVISION} commit was not found in pristine-tar"
   exit 1
fi
# Back to leave the repo in the correct branch
cd ${REPO_PATH}
git checkout ${BRANCH}
echo '# END SECTION'

echo '# BEGIN SECTION: build version and distribution'
sed -i -e "s:\$VERSION:\$OSRF_VERSION:g" debian/changelog

# Use current distro (unstable or experimental are in debian)
changelog_distro=\$(dpkg-parsechangelog | grep Distribution | awk '{print \$2}')
sed -i -e "1 s:\$changelog_distro:$DISTRO:" debian/changelog

if [ -f /usr/bin/rosdep ]; then
  rosdep init
fi

# Do not perform symbol checking
rm -fr debian/*.symbols
echo '# END SECTION'

echo "# BEGIN SECTION: create source package \${OSRF_VERSION}"
rm -f ../*.orig.* ../*.dsc ../*.debian.* ../*.deb ../*.changes ../*.build
# Fix the real problems with lintian and remove true
${GBP_COMMAND} -S || true

cp ../*.dsc $WORKSPACE/pkgs
cp ../*.tar.* $WORKSPACE/pkgs
cp ../*.orig.* $WORKSPACE/pkgs
cp ../*.debian.* $WORKSPACE/pkgs
echo '# END SECTION'

echo '# BEGIN SECTION: create deb packages'
rm -f ../*.deb
${GBP_COMMAND}
echo '# END SECTION'

echo '# BEGIN SECTION: lintian QA'
lintian -I -i ../*.changes || true
lintian -I -i ../*.dsc || true
echo '# END SECTION'

echo '# BEGIN SECTION: export pkgs'
PKGS=\`find ../ -name *.deb || true\`

FOUND_PKG=0
for pkg in \${PKGS}; do
    echo "found \$pkg"
    cp -f \${pkg} $WORKSPACE/pkgs || true
    FOUND_PKG=1
done
# check at least one upload
test \$FOUND_PKG -eq 1 || exit 1
echo '# END SECTION'

${DEBBUILD_AUTOPKGTEST}

echo '# BEGIN SECTION: clean up git build'
cd $REPO_PATH
git reset --hard HEAD
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
		 git-buildpackage \
		 autopkgtest"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
