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

if ${USE_UNSTABLE}; then
   TARGET_DISTRO='unstable'
else
  TARGET_DISTRO='experimental'
  sudo sed -i -e $'s#.*extra_repositories.*#\$extra_repositories = [ \\'deb http://ftp.us.debian.org/debian experimental main\\' ];#' /etc/sbuild/sbuild.conf
fi

echo '# BEGIN SECTION: get source package from experimental'
sudo bash -c 'echo "deb http://deb.debian.org/debian experimental main" >> /etc/apt/sources.list.d/debian-exp.list'
sudo bash -c 'echo "deb-src http://deb.debian.org/debian experimental main" >> /etc/apt/sources.list.d/debian-exp.list'
sudo apt-get update
mkdir /tmp/work
cd /tmp/work
sudo apt-get build-dep -y ${DEB_PACKAGE}
apt-get source -t experimental ${DEB_PACKAGE}
dir=\$(find . -maxdepth 1 -mindepth 1 -type d)
cd \$dir
debuild --no-sign
echo '# END SECTION'

echo '# BEGIN SECTION: create chroot'
sudo sbuild-adduser \${USER}
# experimental should be handled in sbuild.conf file
sudo sbuild-createchroot unstable /srv/chroot/test-amd64-sbuild http://deb.debian.org/debian
echo '# END SECTION'

echo '# BEGIN SECTION: run ratt for ${DEB_PACKAGE}'
cd ..
sed -i -e "s:experimental:unstable:g" ${DEB_PACKAGE}_*.changes
rm -fr ${WORKSPACE}/logs && mkdir ${WORKSPACE}/logs
# use new group to run sbuild
newgrp sbuild << END
echo running ratt under sbuild group
ratt ${DEB_PACKAGE}_*.changes* || echo MARK_AS_UNSTABLE
END
cp -a buildlogs ${WORKSPACE}/logs
echo '# END SECTION'
DELIM

export LINUX_DISTRO=debian
export DISTRO=sid
export DEPENDENCY_PKGS="ratt sbuild quilt devscripts"
export USE_DOCKER_IN_DOCKER=true

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
