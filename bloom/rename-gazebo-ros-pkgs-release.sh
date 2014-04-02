#
# This script should be run on a fork of gazebo_ros_pkgs-release repository
# and will modify control.em bloom templates for:
#
#   - Rename the Package name to $(Package)-current
#   - Define a conflict on the same $(Package)
#
# It will commit and push the hydro and groovy branches
#
# To transform package dependencies among gazebo-ros-pkgs to -current postfix, 
# please use the proper rosdep rules at:
# - https://github.com/osrf/osrf-rosdep/blob/master/00-osrf.list
# 
# Typically the changes lives at osrf/gazebo_ros_pkgs-release-current
# - DONT NEED TO RUN THIS SCRIPT EACH TIME

PKGS="gazebo_plugins gazebo_msgs gazebo_ros gazebo_ros_control gazebo_ros_pkgs"
DISTROS="groovy hydro"

for pkg in ${PKGS}; do
    for distro in ${DISTROS}; do
	if [[ $pkg == "gazebo_ros_control" ]] && [[ $distro == "groovy" ]]; then
	    continue
	fi
	echo " - Processing $pkg in $distro"
        git checkout debian/$distro/$pkg
	# Modify package name
	sed -i -e "s/Package: @(Package)/Package: @(Package)-current/" debian/control.em
	git commit debian/control.em -m "Patch name to release -current version"
	# Include conflict with same package (not current)
        sed -i -e '/^Depends/aConflicts: @(Package)' debian/control.em
	git commit debian/control.em -m "Set up a conflict with same but not -current pkg"
	git push origin debian/$distro/$pkg
    done
done
