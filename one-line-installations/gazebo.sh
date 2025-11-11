#!/bin/sh

# Tabs are strictly needed for indentation of message
# vim: set autoindent noexpandtab ts=4 sw=4 :

set -e

# The installation script is heavy based on get.gazebo.io script
#
# Modfied by jrivero@osrfoundation.org

GZ_VER=11

command_exists() {
	command -v "$@" > /dev/null 2>&1
}

# Check if this is a forked Linux distro
check_forked() {
	# Check for lsb_release command existence, it usually exists in forked distros
	if command_exists lsb_release; then
		# Check if the `-u` option is supported
		set +e
		lsb_release -a -u > /dev/null 2>&1
		lsb_release_exit_code=$?
		set -e

		# Check if the command has exited successfully, it means we're in a forked distro
		if [ "$lsb_release_exit_code" = "0" ]; then
			# Print info about current distro
			cat <<-EOF
			You're using '$lsb_dist' version '$dist_version'.
			EOF

			# Get the upstream release info
			lsb_dist=$(lsb_release -a -u 2>&1 | tr '[:upper:]' '[:lower:]' | grep -E 'id' | cut -d ':' -f 2 | tr -d '[[:space:]]')
			dist_version=$(lsb_release -a -u 2>&1 | tr '[:upper:]' '[:lower:]' | grep -E 'codename' | cut -d ':' -f 2 | tr -d '[[:space:]]')

			# Print info about upstream distro
			cat <<-EOF
			Upstream release is '$lsb_dist' version '$dist_version'.
			EOF
		fi
	fi
}

do_install() {
	cat >&2 <<-'EOF_INIT'
	=======================================================
	!!!! DEPRECATED SCRIPT !!!!
	=======================================================

	This script was installing Gazebo Classic but has been deprecated
	and does not install anything right now. Gazebo Classic was
	replaced by a new series of Gazebo, full documentation is at:

	https://gazebosim.org/docs/latest/gazebo_classic_migration/

	EOF_INIT

	user="$(id -un 2>/dev/null || true)"

	sh_c='sh -c'
	if [ "$user" != 'root' ]; then
		if command_exists sudo; then
			sh_c='sudo -E sh -c'
		elif command_exists su; then
			sh_c='su -c'
		else
			cat >&2 <<-'EOF'
			Error: this installer needs the ability to run commands as root.
			We are unable to find either "sudo" or "su" available to make this happen.
			EOF
			exit 1
		fi
	fi

	curl=''
	if command_exists curl; then
		curl='curl -sSL'
	elif command_exists wget; then
		curl='wget -qO-'
	elif command_exists busybox && busybox --list-modules | grep -q wget; then
		curl='busybox wget -qO-'
	fi

	# check to see which repo they are trying to install from
	repo='main'
	if [ "https://test.gazebo.com/" = "$url" ]; then
		repo='testing'
	elif [ "https://experimental.gazebo.com/" = "$url" ]; then
		repo='experimental'
	fi

	# perform some very rudimentary platform detection
	lsb_dist=''
	dist_version=''
	if command_exists lsb_release; then
		lsb_dist="$(lsb_release -si)"
	fi
	if [ -z "$lsb_dist" ] && [ -r /etc/lsb-release ]; then
		lsb_dist="$(. /etc/lsb-release && echo "$DISTRIB_ID")"
	fi
	if [ -z "$lsb_dist" ] && [ -r /etc/debian_version ]; then
		lsb_dist='debian'
	fi
	if [ -z "$lsb_dist" ] && [ -r /etc/fedora-release ]; then
		lsb_dist='fedora'
	fi
	if [ -z "$lsb_dist" ] && [ -r /etc/oracle-release ]; then
		lsb_dist='oracleserver'
	fi
	if [ -z "$lsb_dist" ]; then
		if [ -r /etc/centos-release ] || [ -r /etc/redhat-release ]; then
			lsb_dist='centos'
		fi
	fi
	if [ -z "$lsb_dist" ] && [ -r /etc/os-release ]; then
		lsb_dist="$(. /etc/os-release && echo "$ID")"
	fi

	lsb_dist="$(echo "$lsb_dist" | tr '[:upper:]' '[:lower:]')"

	if [ -z "$lsb_dist" ] && command_exists sw_vers; then
		lsb_dist='osX'
	fi

	case "$lsb_dist" in
		ubuntu | pop)
			echo "Ubuntu install instructions for the new Gazebo are at:"
			echo "https://gazebosim.org/docs/latest/install_ubuntu/"

		;;

		osX)
			echo "MacOS install instructions for the new Gazebo are at:"
			echo "https://gazebosim.org/docs/latest/install_osx/"
		;;

		debian | oracleserver | fedora | centos)
			echo "There are no specific binary packages for ${lsb_dist}"
			echo "Conda forge provides binaries for gz-sim in multiple platforms:"
			echo "https://github.com/conda-forge/gz-sim-feedstock"
		;;
	esac

	exit 1
}

# wrapped up in a function so that we have some protection against only getting
# half the file during "curl | sh"
do_install
