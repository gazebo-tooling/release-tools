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

# Place in reprepro directory
cd /var/packages/gazebo/ubuntu

upload_package()
{
    local pkg=$1

    sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedeb $DISTRO ${pkg}
    scp -o StrictHostKeyChecking=no -i $HOME/.ssh/id_rsa ${pkg} \
               ubuntu@gazebosim.org:/var/www/assets/distributions
}

pkgs_path="$WORKSPACE/pkgs"

for pkg in `ls $pkgs_path/*.deb`; do
  # Get components from pkg
  pkg=`echo ${pkg} | sed "s:$pkgs_path::"` # remove the root path
  pkg_name=${pkg/_*} # get the first part only
  pkg_suffix=${pkg##*_} # amd64.deb, i386.deb, all.deb
  pkg_version=${pkg#*_} # remove package name
  pkg_version=${pkg_version/_*} # remove package suffix

  case ${pkg_suffix} in
      i386.deb | amd64.deb)
	  upload_package ${pkg}
      ;;
      all.deb)
	# Check if the package already exists. i386 and amd64 generates the same binaries.
	# all should be multiarch, so supposed to work on every platform
	existing_version=$(sudo GNUPGHOME=/var/lib/jenkins/.gnupg/ reprepro ls ${pkg_name} | grep ${DISTRO} | awk '{ print $3 }')
	if [[ ${existing_version} == ${pkg_version} ]]; then
	    echo "${pkg} for ${DISTRO} is already in the repo"
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
