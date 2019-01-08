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

echo '# BEGIN SECTION: create experimental chroot'
sbuild-createchroot experimental /srv/chroot/exp-amd64-sbuild http://deb.debian.org/debian
echo '# END SECTION'

echo '# BEGIN SECTION: run ratt for ${DEB_PACKAGE}'
ratt ${DEB_PACKAGE}
echo '# END SECTION'
DELIM

export LINUX_DISTRO=debian
export DISTRO=sid
export DEPENDENCY_PKGS="ratt sbuild"
. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
