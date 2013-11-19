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
