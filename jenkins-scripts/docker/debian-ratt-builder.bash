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

if ${USE_UNSTABLE}; then
   TARGET_DISTRO='unstable'
else
  TARGET_DISTRO='experimental'
  tee ~/.sbuildrc << EOF
\\\$verbose = 1;
\\\$extra_repositories = [ 'deb http://ftp.us.debian.org/debian experimental main' ];
EOF
fi

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
sudo apt-get build-dep -y ${DEB_PACKAGE}
apt-get source -t experimental ${DEB_PACKAGE}
dir=\$(find . -maxdepth 1 -mindepth 1 -type d)
cd \$dir
debuild --no-sign
echo '# END SECTION'

echo '# BEGIN SECTION: create chroot'
sudo sbuild-adduser \${USER}
# experimental should be handled in sbuild.conf file
if $USE_UNSTABLE; then
  sudo sbuild-createchroot unstable /srv/chroot/test-amd64-sbuild http://deb.debian.org/debian
else
  sudo sbuild-createchroot \
    --extra-repository='deb http://ftp.us.debian.org/debian experimental main' \
    --extra-repository='deb-src http://ftp.us.debian.org/debian experimental main' \
    unstable /srv/chroot/test-amd64-sbuild http://deb.debian.org/debian
fi

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

str_params=''
if [[ -n '$RATT_INCLUDE' ]]; then
  str_params="--include '${RATT_INCLUDE}'"
fi

if [[ -n '$RATT_EXCLUDE' ]]; then
  str_params="\${str_params} --exclude '${RATT_EXCLUDE}'"
fi

# use new group to run sbuild
newgrp sbuild << END
echo running ratt under sbuild group
/tmp/ratt/build \${str_params} ${DEB_PACKAGE}_*.changes* || echo MARK_AS_UNSTABLE
END
cp -a buildlogs ${WORKSPACE}/logs
echo '# END SECTION'
DELIM

export LINUX_DISTRO=debian
if $USE_UNSTABLE; then
  export DISTRO=sid
else
  export DISTRO=experimental
fi
export DEPENDENCY_PKGS="sbuild quilt devscripts dose-extra git"
export USE_DOCKER_IN_DOCKER=true

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
