#!/bin/bash -x
set -ex

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

# S3 Amazon upload
S3_upload()
{
    local pkg=${1} s3_destination_path=${2}

    [[ -z ${pkg} ]] && echo "pkg is empty" && exit 1
    [[ -z ${s3_destination_path} ]] && echo "s3_destination_path is empty" && exit 1

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
    ./s3cmd put $pkg s3://osrf-distributions/${s3_destination_path}
    popd
    rm -fr ${S3_DIR}
}

upload_package()
{
    local pkg=${1}
    [[ -z ${pkg} ]] && echo "Bad parameter pkg" && exit 1

    # Get the canonical package name (i.e. gazebo2 -> gazebo)
    pkg_root_name=${PACKAGE%[[:digit:]]}

    sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedeb $DISTRO ${pkg}

    # The path will end up being: s3://osrf-distributions/$pkg_root_name/releases/
    S3_upload ${pkg} $pkg_root_name/releases/
}

upload_dsc_package()
{
    local pkg=${1}
    [[ -z ${pkg} ]] && echo "Bad parameter pkg" && exit 1

    # .dsc sometimes does not include priority or section, 
    # try to upload and if failed, specify the values
    # see: https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=768046
    sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedsc $DISTRO ${pkg} || \
	sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror --section science --priority extra includedsc $DISTRO ${pkg}
}

upload_zip_file()
{
    local pkg=${1} s3_path=${2}

    [[ -z ${pkg} ]] && echo "Bad parameter pkg" && exit 1
    [[ -z ${s3_path} ]] && echo "Bad parameter s3_path" && exit 1

    S3_upload ${pkg} ${s3_path}
}

pkgs_path="$WORKSPACE/pkgs"

# .zip | (mostly) windows packages
for pkg in `ls $pkgs_path/*.zip`; do
  # S3_UPLOAD_PATH should be send by the upstream job
  if [[ -z ${S3_UPLOAD_PATH} ]]; then
    echo "S3_UPLOAD_PATH was not defined. Not uploading"
    exit 1
  fi
  
  # Seems important to upload the path with a final slash
  upload_zip_file ${pkg} "${S3_UPLOAD_PATH}/"
done

# .dsc | source debian packages
for pkg in `ls $pkgs_path/*.dsc`; do
  upload_dsc_package ${pkg}
done

# .deb | debian packages
for pkg in `ls $pkgs_path/*.deb`; do
  if [[ -z ${DISTRO} ]]; then
    echo 'Error: $DISTRO parameter was not set'
    exit 1
  fi

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

rm -fr $WORKSPACE/pkgs/*.zip
rm -fr $WORKSPACE/pkgs/*.dsc
rm -fr $WORKSPACE/pkgs/*.deb
