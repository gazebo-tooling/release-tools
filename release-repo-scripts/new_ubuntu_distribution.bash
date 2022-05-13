#!/bin/bash

# Create a new distribution inside a -release repo by copying from previous

set -e

if [[ $# -lt 1 ]]; then
    echo "$0 <new_distro>"
    echo " example: $0 jammy"
    exit 1
fi

NEW_DISTRO=${1}

get_latest_distribution_in_dir()
{
  ls | sort | grep -v ubuntu | grep -v debian | tail -1
}

get_source_package_name()
{
  dpkg-parsechangelog -S Source
}

latest_distro=$(get_latest_distribution_in_dir)
if [[ ! -d ${latest_distro} ]]; then
  echo "Failed to detect latest distribution dir: ${latest_distro}"
  exit 1
fi

get_root_repo_dir()
{
  root_check=${PWD}

  if [[ -d ${root_check}/ubuntu ]]; then
    echo "${root_check}"
  elif [[ -d ${root_check}/../ubuntu ]]; then
    echo "${root_check}/../"
  else
    echo "Failed to detect root_dir"
    exit 1
  fi
}

get_relative_path()
{
  common_file=${1}

  pushd debian >/dev/null
  ubuntu_distros="../../ubuntu/debian/${common_file}"
  if [[ -e ${ubuntu_distros} ]]; then
    echo ${ubuntu_distros}
  else
    # debian is two level ups
    echo "../ubuntu/debian/${common_file}"
  fi
  popd >/dev/null
}

list_packages_in_repo()
{
  control_file=${1} src_package=${2}

  grep-dctrl -n -s Package --field Package "${src_package}" ${control_file}  | sort
}

echo -n " + Detecting package release-repo root: "
root_dir=$(get_root_repo_dir)
echo "${root_dir}"
echo -n " + Detecting package source name: "
pushd "${latest_distro}" >/dev/null
source_pkg=$(get_source_package_name)
echo "${source_pkg}"
source_pkg_debian=${source_pkg//[0-9]/}
debian_repo=/tmp/${source_pkg_debian}
echo " + Clone debian repository ${debian_repo}"
if [[ ! -d ${debian_repo} ]]; then
  git clone "https://salsa.debian.org/science-team/${source_pkg_debian}" "${debian_repo}"
fi
popd >/dev/null

echo " + Creating ${NEW_DISTRO} from ${latest_distro}"
cp -a "${latest_distro}" "${NEW_DISTRO}"

pushd "${NEW_DISTRO}" >/dev/null
echo " -- Changes:"
echo "     - Update changelog"
dch --distribution="${NEW_DISTRO}" \
    --force-distribution \
    --check-dirname-level=0 \
    "First release for ${NEW_DISTRO}"
sed -i -e "s:~${latest_distro}.*):~${NEW_DISTRO}):" debian/changelog
head -n 5 debian/changelog > debian/changelog.new
mv debian/changelog.new debian/changelog

echo "     - Remove not needed format file"
rm -f debian/format

echo "     - Import watch file"
cp "${debian_repo}/debian/watch" "${root_dir}/ubuntu/debian/watch"
git add "${root_dir}/ubuntu/debian/watch"
ln -sf $(get_relative_path watch) debian/watch
git add debian/watch

echo "     - Replace Vcs info"
sed -i -e "s:^Vcs-Browser.*:Vcs-Browser\: https\://github.com/gazebosim/${source_pkg_debian}:" \
  "${root_dir}/ubuntu/debian/control"
sed -i -e "s:^Vcs-Hg.*:Vcs-Git\: https\://github.com/gazebosim/${source_pkg_debian}.git:" \
  "${root_dir}/ubuntu/debian/control"
sed -i -e 's:github.com/ignitionrobotics:github.com/gazebosim:g' \
  "${root_dir}/ubuntu/debian/control"

echo "     - Set compat 13"
rm -f debian/compat
echo "13" > debian/compat
git add debian/compat

# Introduce standards. Bionic is in 4.1.4. Focal in 4.5.0
echo "     - Set standards to 4.5.1"
sed -i -e '/Standards-Version:.*/d' "${root_dir}/ubuntu/debian/control"
sed -i -e '/^Homepage: */i Standards-Version: 4.5.1' "${root_dir}/ubuntu/debian/control"
# Bionic has debhelpert 11. Focal and Buster 12.
sed -i -e 's:debhelper (>= 9):debhelper (>= 11):' "${root_dir}/ubuntu/debian/control"

echo "     - Import new copyright"
rm -f debian/copyright
new_copyright_path="${root_dir}/ubuntu/debian/debian_copyright_2022"
cp "${debian_repo}/debian/copyright" "${new_copyright_path}"
git add "${new_copyright_path}"
ln -sf $(get_relative_path copyright) debian/copyright

autopkgtest_path="${root_dir}/ubuntu/debian/tests/"
if [[ ! -d ${autopkgtest_path} ]]; then
  echo "     - Import autopkgtest"
  mkdir -p "${autopkgtest_path}"
  cp -a "${debian_repo}"/debian/tests/* "${autopkgtest_path}"
  git add "${autopkgtest_path}"
  ln -sf $(get_relative_path tests) debian/tests
  git add debian/tests
fi

# Import docs file if found
if [[ -f ${debian_repo}/debian/docs ]]; then
  echo "     - Import docs file"
  cp "${debian_repo}/debian/docs" "${root_dir}/ubuntu/debian/"
  git add "${root_dir}/ubuntu/debian/docs"
  ln -sf $(get_relative_path docs) debian/docs
  git add debian/docs
fi

# Import examples files. Asumming just one file
shopt -s nullglob
for f in "${debian_repo}"/debian/*.examples; do
  if [[ ! -f ${f} ]]; then
    echo "     - Import examples file ${f}"
    cp "${f}" "${root_dir}/ubuntu/debian/"
    git add "${root_dir}"/ubuntu/debian/*.examples
    #ln -sf $(get_relative_path $f) debian/${examples_file}
    # use versioned dev if needed
    if [[ -f $(ls debian/*-dev*.examples) ]]; then
      examples_file=${f##*/}
      mv debian/${examples_file} debian/${examples_file/${source_pkg_debian}/${source_pkg}}
      git add debian/*.examples
    fi
  fi
done

# Check broken symlinks
echo "     - Check broken symlinks"
find debian/ -xtype l
# Finally add new files
git add debian/*

# Info sections
BLUE='\033[1;34m'
NC='\033[0m\n' # No Color
echo
printf "${BLUE}INFO in packages generated${NC}"
or_pkgs_file=/tmp/or_pkgs_file
echo "=Open Robotics=" > ${or_pkgs_file}
list_packages_in_repo "debian/control" >> ${or_pkgs_file}
debian_pkgs_file=/tmp/debian_pkgs_file
echo "=Debian=" > ${debian_pkgs_file}
list_packages_in_repo "${debian_repo}/debian/control" >> ${debian_pkgs_file}
pr -Tm ${or_pkgs_file} ${debian_pkgs_file}
echo
printf "${BLUE}INFO: diff in rules${NC}"
diff -u --color debian/rules "${debian_repo}/debian/rules" || true
echo
printf "${BLUE}INFO: patches in debian${NC}"
ls "${debian_repo}/debian/patches/"
echo
printf "${BLUE}INFO: diff in debian directories${NC}"
diff -qr debian "${debian_repo}/debian" || true
echo
printf "${BLUE}INFO: description in Debian for -dev package${NC}"
grep-dctrl -n -s Description --field=Package "${source_pkg_debian}" "${debian_repo}/debian/control"
