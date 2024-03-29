#!/bin/bash
#
# This script should be run on a fork of gazebo_ros_pkgs-release repository
# and will modify control.em bloom templates for:
#
#   - Rename the Package name to $(Package)X (X being major version)
#   - Define a conflict on current official versiona dn 2 previous major versions $(Package)
#

if [[ ${#} -lt 2 ]]; then
    echo "Usage: ${0} <major_version> <space separted list of rosdistros to release>"
    exit -1
fi

MAJOR_VERSION=${1}
ROS_DISTROS=${*:2}

for i in $(seq 1 2); do
  conflict_ver=$((MAJOR_VERSION - i)) 
  CONFLICTS="${CONFLICTS}, @(Package.replace('gazebo-','gazebo${conflict_ver}-'))"
done

PKGS="gazebo_dev gazebo_plugins gazebo_msgs gazebo_ros gazebo_ros_control gazebo_ros_pkgs"

for pkg in ${PKGS}; do
    for distro in ${ROS_DISTROS}; do
	echo " - Processing $pkg in $distro"
        if ! git checkout "debian/$distro/$pkg"; then
	    if [[ ${pkg} == "gazebo_ros_control" ]]; then
		echo " [??] gazebo_ros_control not found in the repository"
		echo " [??] assuming not avilable in this platform"
		continue
	    fi
	    echo "The branch debian/$distro/$pkg was not found in the repo"
	    echo "Did you forget to run git fetch?"
	    exit 1
	fi
	if grep 'Package.replace' debian/control.em; then
	    echo " + skip ${pkg} for ${distro}: seems to have changes in place"
	    continue
	fi
	# Modify package name
	sed -i -e "s/Package: @(Package)/Package: @(Package.replace('gazebo-','gazebo${MAJOR_VERSION}-'))/" debian/control.em
	sed -i -e "s/Source: @(Package)/Source: @(Package.replace('gazebo-','gazebo${MAJOR_VERSION}-'))/" debian/control.em
	sed -i -e "s/@(Package)/@(Package.replace('gazebo-','gazebo${MAJOR_VERSION}-'))/" debian/changelog.em
	git commit debian/control.em debian/changelog.em -m "Patch name to release ${MAJOR_VERSION} version"
	# Include conflict with same package (not current)
	sed -i -e "/^Depends/aConflicts: @(Package) ${CONFLICTS}" debian/control.em
	git commit debian/control.em -m "Set up a conflict with official ROS packages and the two previous gazebo versions"
	git push origin "debian/$distro/$pkg"
    done
done
