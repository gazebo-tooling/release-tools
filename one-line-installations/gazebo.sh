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
	GAZEBO INSTALLATION SCRIPT
	=======================================================

	This script is installing the latest stable version of
	Gazebo Simulator available from your package manager

	EOF_INIT

	if command_exists gazebo; then
		cat >&2 <<-'EOF'
		Warning: the "gazebo" command appears to already exist on this system.

		If you already have gazebo installed, this script can cause trouble, which is
		why we're displaying this warning and provide the opportunity to cancel the
		installation.

		If you installed the current gazebo package using this script and are using it
		again to update gazebo, you can safely ignore this message.

		You may press Ctrl+C now to abort this script.
		EOF
		( set -x; sleep 20 )
	fi

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

		linuxmint)
			lsb_dist="$(. /etc/os-release && echo "$ID_LIKE")"
			dist_version="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
		;;

		ubuntu | pop)
			if command_exists lsb_release; then
				dist_version="$(lsb_release --codename | cut -f2)"
			fi
			if [ -z "$dist_version" ] && [ -r /etc/lsb-release ]; then
				dist_version="$(. /etc/lsb-release && echo "$DISTRIB_CODENAME")"
			fi
			case "$dist_version" in
				jammy)
					# Packages for Jammy come directly from Ubuntu repositories, unversioned
					# No released packages in packages.o.o
					GZ_VER=
				;;
				xenial)
					GZ_VER=10
				;;
				artful | eoan )
					GZ_VER=9
				;;
			esac
		;;

		debian)
			dist_version="$(cat /etc/debian_version | sed 's/\/.*//' | sed 's/\..*//')"
			case "$dist_version" in
				10)
					dist_version="buster"
					GZ_VER=9
				;;

				9)
					dist_version="stretch"
					GZ_VER=7
			    ;;
				8)
					dist_version="jessie"
					GZ_VER=7
				;;
			esac
		;;

		oracleserver)
			# need to switch lsb_dist to match yum repo URL
			lsb_dist="oraclelinux"
			dist_version="$(rpm -q --whatprovides redhat-release --queryformat "%{VERSION}\n" | sed 's/\/.*//' | sed 's/\..*//' | sed 's/Server*//')"
		;;

		fedora|centos)
			dist_version="$(rpm -q --whatprovides redhat-release --queryformat "%{VERSION}\n" | sed 's/\/.*//' | sed 's/\..*//' | sed 's/Server*//')"
		;;

		osX)
			full_major_version="$(sw_vers -productVersion | sed 's:\.[0-9]*$::')"
			# Check for supported versions
			case "$full_major_version" in
				10.10)
					dist_version="yosemite"
				;;
				10.11)
					dist_version="elcapitan"
				;;
			esac
		;;

		*)
			if command_exists lsb_release; then
				dist_version="$(lsb_release --codename | cut -f2)"
			fi
			if [ -z "$dist_version" ] && [ -r /etc/os-release ]; then
				dist_version="$(. /etc/os-release && echo "$VERSION_ID")"
			fi
		;;


	esac

	# Check if this is a forked Linux distro
	check_forked

	# Run setup for each distro accordingly
	case "$lsb_dist" in
		amzn)
			(
			set -x
			$sh_c 'sleep 3; yum -y -q install gazebo-devel'
			)
			echo_gazebo_as_nonroot
			exit 0
			;;
		debian | ubuntu | pop)
			export DEBIAN_FRONTEND=noninteractive
			DEB_PKG_NAME="libgazebo$GZ_VER-dev gazebo$GZ_VER"

			cat >&2 <<-'EOF'

			In Debian this script will setup the osrfoundation
			repository to install the latest package available

			EOF

			did_apt_get_update=
			apt_get_update() {
				if [ -z "$did_apt_get_update" ]; then
					( set -x; $sh_c 'sleep 3; apt-get update' )
					did_apt_get_update=1
				fi
			}

			(
			set -x
			$sh_c "apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743"
			$sh_c "mkdir -p /etc/apt/sources.list.d"
			$sh_c "echo deb http://packages.osrfoundation.org/gazebo/$lsb_dist\-stable $dist_version main > /etc/apt/sources.list.d/gazebo-stable.list"
			$sh_c "sleep 3; apt-get update; apt-get install -y -q $DEB_PKG_NAME"
			)
			exit 0
			;;
		fedora)
			(
				  set -x
				  $sh_c 'sleep 3; dnf -y -q install gazebo-devel'
			)
			exit 0
			;;
		gentoo)
			# In Gentoo, all gazebo versions are currently masked ~arch
			# TODO: update when going stable
			# TODO: are all dependencies stable?
			echo " * Using the unstable version of gazebo from ~arch"
			echo "sci-electronics/gazebo" >> /etc/portage/package.accept_keywords
			$sh_c 'sleep 3; emerge sci-electronics/gazebo'
			exit 0
			;;
		osX)
			BREW_PKG_NAME=gazebo${GZ_VER}
			(
			  if ! command_exists ruby; then
				echo "ERROR: ruby executable is not found in your system path."
				echo "Please check your installation."
				exit 1
			  fi

			  export PATH=/usr/local/bin:/usr/local/sbin:${PATH}
			  if ! command_exists brew; then
				echo "Installing Homebrew:"
				ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
				echo "Homebrew installation complete."
				echo
			  fi

			  if ! pkgutil --pkg-info org.macosforge.xquartz.pkg; then
				if ! pkgutil --pkg-info org.xquartz.X11; then
				  echo "Installing XQuartz:"
				  brew install homebrew/cask/xquartz
				  echo "XQuartz installation complete."
				  echo
				fi
			  fi

			  brew tap osrf/simulation
			  brew update
			  brew install ${BREW_PKG_NAME}
			  brew audit ${BREW_PKG_NAME} || true
			  brew test ${BREW_PKG_NAME}
			  brew doctor
			)

			exit 0
			;;
	esac

	# intentionally mixed spaces and tabs here -- tabs are stripped by "<<-'EOF'", spaces are kept in the output
	cat >&2 <<-'EOF_END'

	Either your platform is not easily detectable, is not supported by this
	installer script (yet - PRs welcome! [https://github.com/gazebo-tooling/release-tools])
    or does not yet have a package for gazebo.  Please visit the following URL for more detailed
	installation instructions:

      http://gazebosim.org/tutorials?cat=install

	EOF_END
	exit 1
}

# wrapped up in a function so that we have some protection against only getting
# half the file during "curl | sh"
do_install
