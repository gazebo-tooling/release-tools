#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

echo '# BEGIN SECTION: setup the testing enviroment'
# Define the name to be used in docker
export DOCKER_JOB_NAME="debian_ratt_builder"
. "${SCRIPT_DIR}/lib/boilerplate_prepare.sh"
. "${SCRIPT_DIR}/lib/_common_scripts.bash"
echo '# END SECTION'

cat > build.sh << DELIM
$(generate_buildsh_header)

sudo rm /var/lib/apt/lists/*.lz4
sudo sed -i -e 's:GzipIndexes "true":GzipIndexes "false":g' /etc/apt/apt.conf.d/*
sudo apt-get update
sudo ls -las /var/lib/apt/lists/*

echo '# BEGIN SECTION: get source package from experimental'
sudo bash -c 'echo "deb http://deb.debian.org/debian experimental main" >> /etc/apt/sources.list.d/debian-exp.list'
sudo bash -c 'echo "deb-src http://deb.debian.org/debian experimental main" >> /etc/apt/sources.list.d/debian-exp.list'
sudo apt-get update
mkdir /tmp/work
cd /tmp/work
apt-get source -t experimental ${DEB_PACKAGE}
dir=\$(find . -maxdepth 1 -mindepth 1 -type d)
# Install the build dependencies through a metapackage, so that they can be
# removed before the reverse dependencies are built in this same container
mkdir /tmp/build-deps
pushd /tmp/build-deps 2> /dev/null
sudo mk-build-deps -i -r -t 'apt-get -y --no-install-recommends' /tmp/work/\$dir/debian/control
popd 2> /dev/null
cd \$dir
debuild --no-sign
echo '# END SECTION'

echo '# BEGIN SECTION: remove the build dependencies of ${DEB_PACKAGE}'
# sbuild builds every reverse dependency directly in this container, not in
# a clean chroot. It purges what it installs for each build, but anything
# already installed here stays visible to all of them and could hide a
# missing Build-Depends. Leave the container as close to its initial
# package set as possible.
sudo apt-get -y purge ${DEB_PACKAGE}-build-deps
sudo apt-get -y autoremove --purge
echo '# END SECTION'

echo '# BEGIN SECTION: configure sbuild'
# Creating an sbuild chroot (schroot or unshare) needs a privileged
# container. Instead, sbuild builds every reverse dependency directly in
# this container through the autopkgtest null server, purging the build
# dependencies after each build. That mode needs sbuild to run as root;
# dpkg-buildpackage still runs as the unprivileged sbuild user.
sudo mkdir -p /root/.config/sbuild
sudo tee /root/.config/sbuild/config.pl << EOF
\\\$chroot_mode = 'autopkgtest';
\\\$autopkgtest_virt_server = 'null';
\\\$build_user = 'sbuild';
EOF
if ! $USE_UNSTABLE; then
  sudo tee -a /root/.config/sbuild/config.pl << EOF
\\\$verbose = 1;
\\\$extra_repositories = [ 'deb http://ftp.us.debian.org/debian experimental main' ];
EOF
fi
echo '# END SECTION'

echo '# BEGIN SECTION: check the sbuild setup'
# ratt reports every failing sbuild call as a broken reverse dependency,
# which only marks the job as unstable. Build a dummy package first so that
# a broken sbuild setup fails the job instead.
mkdir -p /tmp/sbuild-check/sbuild-check/debian/source
pushd /tmp/sbuild-check 2> /dev/null
echo '3.0 (native)' > sbuild-check/debian/source/format
cat > sbuild-check/debian/control << 'EOF'
Source: sbuild-check
Section: misc
Priority: optional
Maintainer: OSRF Jenkins <build@osrfoundation.org>
Build-Depends: debhelper-compat (= 13)
Standards-Version: 4.6.2

Package: sbuild-check
Architecture: any
Description: dummy package to check the sbuild setup
 Built before running ratt so that a broken sbuild setup fails the job.
EOF
cat > sbuild-check/debian/changelog << 'EOF'
sbuild-check (1.0) unstable; urgency=medium

  * Check the sbuild setup.

 -- OSRF Jenkins <build@osrfoundation.org>  Mon, 01 Jan 2024 00:00:00 +0000
EOF
printf '#!/usr/bin/make -f\n%%:\n\tdh \$@\n' > sbuild-check/debian/rules
chmod +x sbuild-check/debian/rules
dpkg-source -b sbuild-check
sudo -H sbuild --nolog --no-run-lintian --no-run-autopkgtest --no-run-piuparts \
  --dist=unstable sbuild-check_1.0.dsc
popd 2> /dev/null
# sbuild leaves the apt lists without the Packages and Sources indexes,
# which ratt reads to find the reverse dependencies
sudo apt-get update
echo '# END SECTION'

echo '# BEGIN SECTION: run ratt for ${DEB_PACKAGE}'
cd ..
if $USE_UNSTABLE; then
  sed -i -e "s:experimental:unstable:g" ${DEB_PACKAGE}_*.changes
fi
rm -fr ${WORKSPACE}/logs && mkdir ${WORKSPACE}/logs

sudo apt-get install -y golang-go
pushd /tmp 2> /dev/null
git clone https://github.com/j-rivero/ratt
cd ratt
go mod init local/build
go mod tidy
go build
popd 2> /dev/null

# An array keeps each filter as a single argument, without the quotes
ratt_params=()
if [[ -n '$RATT_INCLUDE' ]]; then
  ratt_params+=(--include '${RATT_INCLUDE}')
fi

if [[ -n '$RATT_EXCLUDE' ]]; then
  ratt_params+=(--exclude '${RATT_EXCLUDE}')
fi

# sbuild needs root, see the sbuild configuration above
sudo -H /tmp/ratt/build "\${ratt_params[@]}" ${DEB_PACKAGE}_*.changes* || echo MARK_AS_UNSTABLE
sudo chown -R \${USER}: buildlogs
cp -a buildlogs ${WORKSPACE}/logs
echo '# END SECTION'
DELIM

export LINUX_DISTRO=debian
if $USE_UNSTABLE; then
  export DISTRO=sid
else
  export DISTRO=experimental
fi
export DEPENDENCY_PKGS="sbuild autopkgtest quilt devscripts equivs dose-extra git"

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
