#!/bin/bash -x
set -ex

if [[ -z ${DISTRO} ]]; then
    echo 'Error: $DISTRO parameter was not set'
    exit 1
fi

NEEDED_HOST_PACKAGES="reprepro openssh-client"
QUERY_HOST_PACKAGES=$(dpkg-query -Wf'${db:Status-abbrev}' ${NEEDED_HOST_PACKAGES} 2>&1) || true
if [[ -n ${QUERY_HOST_PACKAGES} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

# Check if the node was configured to use s3cmd
# This is done by running s3cmd --configure
if [[ ! -f "${HOME}/.s3cfg" ]]; then
    echo "No $HOME/.s3cfg file found. Please config the software first in your system"
    exit 1
fi

# Place in reprepro directory
cd /var/packages/gazebo/ubuntu

upload_package()
{
    local pkg=$1

    # Get the canonical package name (i.e. gazebo2 -> gazebo)
    pkg_root_name=${PACKAGE%[[:digit:]]}

    sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedeb $DISTRO ${pkg}

    # S3 Amazon upload
    S3_DIR=$(mktemp -d ${HOME}/s3.XXXX)
    pushd ${S3_DIR}
    # Hack for not failing when github is down
    update_done=false
    seconds_waiting=0
    while (! $update_done); do
      wget https://github.com/s3tools/s3cmd/archive/v1.5.0-rc1.tar.gz -O foo.tar.gz && update_done=true
      sleep 1
      seconds_waiting=$((seconds_waiting+1))
      [ $seconds_waiting -gt 60 ] && exit 1
    done
    tar xzf foo.tar.gz
    cd s3cmd-*
    ./s3cmd put $pkg s3://osrf-distributions/$pkg_root_name/releases/
    popd
    rm -fr ${S3_DIR}
}

upload_dsc_package()
{
    sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedsc $DISTRO ${pkg}
}

pkgs_path="$WORKSPACE/pkgs"

for pkg in `ls $pkgs_path/*.dsc`; do
  upload_dsc_package ${pkg}
done

for pkg in `ls $pkgs_path/*.deb`; do
  # Get components from pkg
  pkg_relative=`echo ${pkg} | sed "s:$pkgs_path/::"` # remove the root path
  pkg_name=${pkg_relative/_*} # get the first part only
  pkg_suffix=${pkg_relative##*_} # amd64.deb, i386.deb, all.deb
  pkg_version=${pkg_relative#*_} # remove package name
  pkg_version=${pkg_version/_*} # remove package suffix

  case ${pkg_suffix} in
      i386.deb | amd64.deb | armhf.deb)
	  upload_package ${pkg}
      ;;
      all.deb)
	# Check if the package already exists. i386 and amd64 generates the same binaries.
	# all should be multiarch, so supposed to work on every platform
	existing_version=$(sudo GNUPGHOME=/var/lib/jenkins/.gnupg/ reprepro ls ${pkg_name} | grep ${DISTRO} | awk '{ print $3 }')
	if [[ ${existing_version} == ${pkg_version} ]]; then
	    echo "${pkg_relative} for ${DISTRO} is already in the repo"
	    echo "SKIP UPLOAD"
	    continue
	fi
	upload_package ${pkg}
      ;;
      *)
	  echo "ERROR: unknown pkg_suffix: ${pkg_suffix}"
	  exit 1
      ;;
  esac
done

rm -fr $WORKSPACE/pkgs/*.deb
rm -fr $WORKSPACE/pkgs/*.dsc
