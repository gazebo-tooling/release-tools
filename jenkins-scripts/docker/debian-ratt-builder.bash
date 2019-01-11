#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
export DOCKER_JOB_NAME="debian_ratt_builder"
. "${SCRIPT_DIR}/lib/boilerplate_prepare.sh"
echo '# END SECTION'

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
set -ex


echo '# BEGIN SECTION: get source package from experimental'
echo "deb http://deb.debian.org/debian experimental main" >> /etc/apt/sources.list
echo "deb-src http://deb.debian.org/debian experimental main" >> /etc/apt/sources.list
apt-get update
mkdir /tmp/work
cd /tmp/work
apt-get build-dep -y ${DEB_PACKAGE}
apt-get source -t experimental ${DEB_PACKAGE}
dir=\$(find . -maxdepth 1 -mindepth 1 -type d)
cd \$dir
debuild -S --no-sign
echo '# END SECTION'

echo '# BEGIN SECTION: create experimental chroot'
sbuild-createchroot unstable /srv/chroot/exp-amd64-sbuild http://deb.debian.org/debian
echo '# END SECTION'

echo '# BEGIN SECTION: run ratt for ${DEB_PACKAGE}'
cd ..
# need to configure unstable in the change file, not all packages are in experimental
sed -i -e 's:experimental:unstable:g' ${DEB_PACKAGE}_*.changes
ratt ${DEB_PACKAGE}_*.changes*
echo '# END SECTION'
DELIM

export LINUX_DISTRO=debian
export DISTRO=sid
export DEPENDENCY_PKGS="ratt sbuild quilt devscripts"

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
