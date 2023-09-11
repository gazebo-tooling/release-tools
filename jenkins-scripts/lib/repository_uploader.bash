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
    s3cmd put $pkg s3://osrf-distributions/${s3_destination_path}
    popd
    rm -fr ${S3_DIR}
}

# Check if the dsc package (include .orig.tar.gz) is alrady in the repo
# The dsc package is valid across all distributions
dsc_package_exists_and_equal_or_greater()
{
    local pkg=${1} new_version=${2} distro=${3}

    current_dsc_info=$(GNUPGHOME=$HOME/.gnupg/ reprepro -T dsc list "${distro}" "${pkg}" | tail -n 1)

    if [[ -z ${current_dsc_info} ]]; then
        return 1 # do not exits, false
    fi

    current_version=$(awk '{ print $3 }' <<< "${current_dsc_info}")
    # if new version is lower
    if $(dpkg --compare-versions ${new_version} gt ${current_version}); then
        return 1 # exists, new package is greater
    fi

    return 0 # exist and current package is greater or equal than new one
}

upload_package()
{
    local pkg=${1} pkg_name=${2}
    [[ -z ${pkg} ]] && echo "Bad parameter pkg" && exit 1
    [[ -z ${pkg_name} ]] && echo "Bad parameter pkg_name" && exit 1

    GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedeb $DISTRO ${pkg}

    # The path will end up being: s3://osrf-distributions/$pkg_root_name/releases/
    S3_upload ${pkg} ${pkg_name}/releases/
}

upload_dsc_package()
{
    local pkg=${1}
    [[ -z ${pkg} ]] && echo "Bad parameter pkg" && exit 1

    # .dsc sometimes does not include priority or section,
    # try to upload and if failed, specify the values
    # see: https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=768046
    GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror includedsc $DISTRO ${pkg} || \
	  GNUPGHOME=$HOME/.gnupg reprepro --nothingiserror --section science --priority extra includedsc $DISTRO ${pkg} || \
		echo "MARK_BUILD_UNSTABLE"
}

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
    echo "No UPLOAD_TO_REPO value was send. Which repository to use? (stable | prerelease | nightly | none)"
    echo ""
    echo "Please check that the jenkins -debbuild job that called this uploader is defining the parameter"
    echo "UPLOAD_TO_REPO in the job configuration. If it is not, just define it as a string parameter"
    exit 1
fi

case ${UPLOAD_TO_REPO} in
  "stable")
    # Security checks not to upload nightly or prereleases
    # No packages with ~git or ~pre
    if [[ -n $(ls ${pkgs_path}/*~git*.*) && -n $(ls ${pkgs_path}/*~pre*.*) ]]; then
      echo "There are nightly packages in the upload directory. Not uploading to stable repo"
      exit 1
    fi
          # No source packages with ~git in version
    if [[ -n $(cat ${pkgs_path}/*.dsc | grep ^Version: | grep '~git\|~pre') ]]; then
            echo "There is a sorce package with nightly or pre in version. Not uploading to stable repo"
      exit 1
    fi
  ;;
  "nightly")
    # No uploads for nightly packages test runs
    ENABLE_S3_UPLOAD=false
  ;;
  "none")
    echo "UPLOAD_TO_REPO was set to none. No upload is done."
    exit 0
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


# S3_FILES_TO_UPLOAD can contain a list of filenames relative to $pkgs_path that will be uploaded to S3
# together with any .zip (mostly old windows packages) if they exists
S3_FILES_TO_UPLOAD="${S3_FILES_TO_UPLOAD} $(find "$pkgs_path" -type f -name '*.zip' -printf '%f\n' || true)"

for pkg in ${S3_FILES_TO_UPLOAD}; do
  # S3_UPLOAD_PATH should be send by the upstream job
  if [[ -z ${S3_UPLOAD_PATH} ]]; then
    echo "S3_UPLOAD_PATH was not defined. Not uploading"
    exit 1
  fi

  # Seems important to upload the path with a final slash
  S3_upload "${pkgs_path}/${pkg}" "${S3_UPLOAD_PATH}"
done

# .bottle | brew binaries
for pkg in `find "$pkgs_path" -name '*.bottle*.json'`; do
  # Extract bottle name and root_url from json file
  bottle_filename=$(urlencode -d $(dirname $pkg)/$(jq -r '.[]["bottle"]["tags"][]["filename"]' < $pkg))
  root_url=$(jq -r '.[]["bottle"]["root_url"]' < $pkg)
  s3_directory=${root_url#https://osrf-distributions\.s3\.amazonaws\.com/}

  if [[ -z ${s3_directory} ]]; then
    echo "Failed to infer s3 directory from bottle filename: ${pkg}"
    exit 1
  fi

  # Seems important to upload the path with a final slash
  s3_directory_one_slash=${s3_directory%%*(/)}/
  S3_upload ${bottle_filename} "${s3_directory_one_slash}"
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
  pkg_name_clean=${pkg##*/}
  pkg_name=${pkg_name_clean/_*}
  pkg_version=${pkg_name_clean#*_}
  pkg_version=${pkg_version/.dsc}

  if dsc_package_exists_and_equal_or_greater ${pkg_name} ${pkg_version} ${DISTRO}; then
    echo "Source package for ${pkg} already exists in the repo and it's greater or equal than current version"
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
      i386.deb | amd64.deb | armhf.deb | arm64.deb)
	  upload_package ${pkg} ${PACKAGE_ALIAS}
      ;;
      all.deb)
	# Check if the package already exists. i386 and amd64 generates the same binaries.
	# all should be multiarch, so supposed to work on every platform
	existing_version=$(GNUPGHOME=$HOME/.gnupg/ reprepro ls ${pkg_name} | grep ${DISTRO} | awk '{ print $3 }')
	if $(dpkg --compare-versions ${pkg_version} le ${existing_version}); then
	    echo "${pkg_relative} for ${DISTRO} is already in the repo with same version or greater"
	    echo "SKIP UPLOAD"
	    continue
	fi
	upload_package ${pkg} ${PACKAGE_ALIAS}
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
