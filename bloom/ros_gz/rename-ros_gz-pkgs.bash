#!/bin/bash
#
# This script should be run on a fork of ros_ign-release/ros_gz-release repository
# and will modify control.em bloom templates for:
#
#   - Rename the Package name modifying -gz- by -gz$DISTRO-
#   - Define a conflict on current official name: $(Package)
#

if [[ ${#} -lt 2 ]]; then
  echo "Usage: ${0} <gz_release_to_use>> <space separted list of rosdistros to release>"
  exit 1
fi

# Safety check for ros2-gbp repo
if [[ -n $(git config --get remote.origin.url | grep git@github.com:ros2-gbp/ros_ign-release.git) ]]; then
  echo "This script refuses to modify the ros2-gbp repository. You probably don't want this."
  exit 1
fi

GZ_RELEASE=${1}
ROS_DISTROS=${*:2}

PKGS="ros_gz ros_gz_bridge ros_gz_image ros_gz_interfaces ros_gz_sim ros_gz_sim_demos"

for pkg in ${PKGS}; do
  for distro in ${ROS_DISTROS}; do
    echo " - Processing $pkg in $distro"
      if ! git checkout "debian/$distro/$pkg"; then
      echo "The branch debian/$distro/$pkg was not found in the repo"
      echo "Did you forget to run git fetch?"
      exit 1
    fi
    # Add GZ_VERSION env variable in rules file
    sed -i -e "/export DH_VERBOSE/a\export GZ_VERSION=${GZ_RELEASE}" debian/rules.em
    git commit debian/rules.em -m "Export GZ_VERSION in rules file"
    git push origin "debian/$distro/$pkg"
    if grep 'Package.replace' debian/control.em; then
      echo " + skip ${pkg} for ${distro}: seems to have changes in place"
      continue
    fi
    # Modify package name. Note that regexp can not be stricker than
    # *-gz since there is a package named package ros-gz
    sed -i -e "s/Package: @(Package)/Package: @(Package.replace('-gz','-gz${GZ_RELEASE}'))/" debian/control.em
    sed -i -e "s/Source: @(Package)/Source: @(Package.replace('-gz','-gz${GZ_RELEASE}'))/" debian/control.em
    sed -i -e "s/@(Package)/@(Package.replace('-gz','-gz${GZ_RELEASE}'))/" debian/changelog.em
    git commit debian/control.em debian/changelog.em -m "Patch name to release ${GZ_RELEASE} version"
    # Include conflict with initial package name in ROS
    sed -i -e '/^Depends/a\Conflicts: \@(Package)' debian/control.em
    if [[ -n ${GZ_RELEASE_TO_CONFLICT} ]]; then
      sed -i -e "s/Conflicts: @(Package)/Conflicts: @(Package), @(Package.replace('-gz','-gz${GZ_RELEASE_TO_CONFLICT}'))/" debian/control.em
    fi
    git commit debian/control.em -m "Set up a conflict with official ROS packages"
    git push origin "debian/$distro/$pkg"
  done
done
