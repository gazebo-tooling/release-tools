#!/bin/bash -x
set -ex

# S3 Amazon upload
S3_upload()
{
    local pkg=${1} s3_destination_path=${2}

    if ! $ENABLE_S3_UPLOAD; then
        echo '# BEGIN SECTION: S3 upload is DISABLED'
	echo "S3 upload is disabled"
	echo '# END SECTION'
	return
    fi

    [[ -z ${pkg} ]] && echo "pkg is empty" && exit 1
    [[ -z ${s3_destination_path} ]] && echo "s3_destination_path is empty" && exit 1

    # handle canonical paths if needed
    if $S3_UPLOAD_CANONICAL_PATH; then
      s3_destination_path=$(sed -e 's@[[:digit:]]*$@@' <<< "${s3_destination_path}")
      # S3_UPLOAD_PATH can be composed by "pkg1/releases/"
      s3_destination_path=$(sed -e 's@[[:digit:]]*/@/@' <<< "${s3_destination_path}")
    fi

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

dsc_package_exists()
{
    local pkg=${1} # name, no full path

    if [[ -n $(sudo GNUPGHOME=/var/lib/jenkins/.gnupg/ reprepro ls ${pkg} | grep source) ]]; then
	return 0 # exists, true
    fi

    return 1 # do not exits, false
}

upload_package()
{
    local pkg=${1}
    [[ -z ${pkg} ]] && echo "Bad parameter pkg" && exit 1

    sudo GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedeb $DISTRO ${pkg}

    # The path will end up being: s3://osrf-distributions/$pkg_root_name/releases/
    S3_upload ${pkg} ${pkg}/releases/
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

NEEDED_HOST_PACKAGES="reprepro openssh-client"
QUERY_HOST_PACKAGES=$(dpkg-query -Wf'${db:Status-abbrev}' ${NEEDED_HOST_PACKAGES} 2>&1) || true
if [[ -n ${QUERY_HOST_PACKAGES} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

# By default, enable s3 upload of packages
ENABLE_S3_UPLOAD=true

# PATH to packages
pkgs_path="$WORKSPACE/pkgs"

# Check if the node was configured to use s3cmd
# This is done by running s3cmd --configure
if [[ ! -f "${HOME}/.s3cfg" ]]; then
    echo "No $HOME/.s3cfg file found. Please config the software first in your system"
    exit 1
fi

# Check destination repository
if [[ -z ${UPLOAD_TO_REPO} ]]; then
    echo "No UPLOAD_TO_REPO value was send. Which repository to use? (stable | prerelease | nightly)"
    echo ""
    echo "Please check that the jenkins -debbuild job that called this uploader is defining the parameter"
    echo "UPLOAD_TO_REPO in the job configuration. If it is not, just define it as a string parameter"
    exit 1
fi

case ${UPLOAD_TO_REPO} in
    "stable")
	# Security checks not to upload nightly or prereleases
        # No packages with ~hg or ~pre
	if [[ -n $(ls ${pkgs_path}/*~hg*.*) && -n $(ls ${pkgs_path}/*~pre*.*) ]]; then
	  echo "There are nightly packages in the upload directory. Not uploading to stable repo"
	  exit 1
	fi
        # No source packages with ~hg in version
	if [[ -n $(cat ${pkgs_path}/*.dsc | grep ^Version: | grep '~hg\|~pre') ]]; then
          echo "There is a sorce package with nightly or pre in version. Not uploading to stable repo"
	  exit 1
        fi
	;;
    "nightly")
	# No uploads for nightly packages
	ENABLE_S3_UPLOAD=false
	;;
    "only_s3_upload")
        # This should be fine, no repo, only s3 upload
        ENABLE_S3_UPLOAD=true
        ;;
    *)
	# Here we could find project repositories uploads or error values.
	# Error values for UPLOAD_TO_REPO will be get in the next directory check
	# some lines below so we do nothing.
	;;
esac

# .zip | (mostly) windows packages
for pkg in `ls $pkgs_path/*.zip`; do
  # S3_UPLOAD_PATH should be send by the upstream job
  if [[ -z ${S3_UPLOAD_PATH} ]]; then
    echo "S3_UPLOAD_PATH was not defined. Not uploading"
    exit 1
  fi
  
  # Seems important to upload the path with a final slash
  S3_upload ${pkg} "${S3_UPLOAD_PATH}"
done

# .bottle | brew binaries
for pkg in `ls $pkgs_path/*.bottle.tar.gz`; do
  # There could be more than one bottle exported so do not relay on variables
  # and extract information from bottles filenames
  pkg_filename=${pkg##*/} # leave file name only
  pkg_name=${pkg_filename%-*} # remove from the last - until the end
  pkg_canonical_name=${pkg_name/[0-9]*} # remove all version from name
  s3_directory=${pkg_canonical_name/ignition/ign} # use short version ignition

  if [[ -z ${s3_directory} ]]; then
    echo "Failed to infer s3 directory from bottle filename: ${pkg}"
    exit 1
  fi
  
  # Seems important to upload the path with a final slash
  S3_upload ${pkg} "${s3_directory}/releases/"
done

# Check for no reprepro uploads to finish here
if [[ ${UPLOAD_TO_REPO} == 'only_s3_upload' ]]; then
  exit 0
fi

# REPREPRO debian packages
LINUX_DISTRO=${LINUX_DISTRO:-ubuntu}
repo_path="/var/packages/gazebo/${LINUX_DISTRO}-${UPLOAD_TO_REPO}"

if [[ ! -d ${repo_path} ]]; then
    echo "Repo directory ${repo_path} not found in server"
    exit 1
fi

# Place in reprepro directory
cd ${repo_path}

# .dsc | source debian packages
for pkg in `ls $pkgs_path/*.dsc`; do
  pkg_name=${pkg##*/} 
  pkg_name=${pkg_name/_*}    

  if dsc_package_exists ${pkg_name}; then
    echo "Source package for ${pkg} already exists in the repo"
    echo "SKIP SOURCE UPLOAD"
  else
    upload_dsc_package ${pkg}
  fi
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
	if $(dpkg --compare-versions ${pkg_version} le ${existing_version}); then
	    echo "${pkg_relative} for ${DISTRO} is already in the repo with same version or greater"
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
