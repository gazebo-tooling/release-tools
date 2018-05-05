#!/bin/sh
set -e

# The installation script is heavy based on get.docker.io script
#
# Modfied by jrivero@osrfoundation.org

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
        # respect the tabs for proper EOF and formatting
	cat <<-EOF
	You're using '$lsb_dist' version '$dist_version'.
	EOF

	# Get the upstream release info
	lsb_dist=$(lsb_release -a -u 2>&1 | tr '[:upper:]' '[:lower:]' | grep -E 'id' | cut -d ':' -f 2 | tr -d '[[:space:]]')
	dist_version=$(lsb_release -a -u 2>&1 | tr '[:upper:]' '[:lower:]' | grep -E 'codename' | cut -d ':' -f 2 | tr -d '[[:space:]]')

        # respect the tabs for proper EOF and formatting
	cat <<-EOF
	Upstream release is '$lsb_dist' version '$dist_version'.
	EOF
    fi
  fi
}

do_install() {
  if command_exists gazebo; then
    # respect the tabs for proper EOF and formatting
	cat >&2 <<-'EOF'
		Warning: the "gazebo" command appears to already exist on this system.

		If you already have gazebo installed, this script can cause trouble, which is
		why we're displaying this warning and provide the opportunity to cancel the
		installation.

		If you installed the current SRCSim gazebo package using this script and
		are using it again to update the simulator, you can safely ignore this message.

		You may press Ctrl+C now to abort this script.
	EOF
#	( set -x; sleep 20 )
  fi

  user="$(id -un 2>/dev/null || true)"

  sh_c='sh -c'
  if [ "$user" != 'root' ]; then
    if command_exists sudo; then
      sh_c='sudo -E sh -c'
    elif command_exists su; then
      sh_c='su -c'
    else
        # respect the tabs for proper EOF and formatting
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

  case "$lsb_dist" in

    ubuntu)
      if command_exists lsb_release; then
        dist_version="$(lsb_release --codename | cut -f2)"
      fi
      if [ -z "$dist_version" ] && [ -r /etc/lsb-release ]; then
        dist_version="$(. /etc/lsb-release && echo "$DISTRIB_CODENAME")"
      fi
    ;;

    debian)
      dist_version="$(cat /etc/debian_version | sed 's/\/.*//' | sed 's/\..*//')"
      case "$dist_version" in
        8)
          dist_version="jessie"
        ;;
        7)
          dist_version="wheezy"
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
    ubuntu)
      case "$dist_version" in
        trusty)
          did_apt_get_update=
          apt_get_update() {
            if [ -z "$did_apt_get_update" ]; then
              ( set -x; $sh_c 'sleep 3; apt-get update' )
              did_apt_get_update=1
            fi
          }

          if [ -z "$curl" ]; then
            apt_get_update
            ( set -x; $sh_c 'sleep 3; apt-get install -y -q curl ca-certificates' )
            curl='curl -sSL'
          fi

          (
          set -x
          # intentionally mixed spaces and tabs here -- tabs are stripped by "<<-'EOF_INSTALL'", spaces are kept in the output
			cat >&2 <<-EOF_INSTALL

			  This install process will install all the packages needed to run
			  SRCSim but also configure the user account $USER to be able to run 
			  the simulation out-of-the-box. This implies to do the following
			  changes in the user $HOME:

			   - Install gazebo models under $HOME/.gazebo/models
			   - Install valkyrie controller under $HOME/valkyrie

			   If you do not agree with these actions, please control+C now

			EOF_INSTALL
		  ( set -x; sleep 20 )
          # 0. Safety Checks
          if [ -z $USER ]; then
            echo 'The $USER env variable is empty, it should not'
            exit 1
          fi
	  # 1. Install repositories
          $sh_c 'apt-get update; apt-get install -y -q wget'
          $sh_c "mkdir -p /etc/apt/sources.list.d"
          $sh_c "apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743"
          $sh_c "echo deb http://packages.osrfoundation.org/gazebo/${lsb_dist} ${dist_version} main > /etc/apt/sources.list.d/gazebo-stable.list"
	  $sh_c "wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -"
          $sh_c "echo deb http://srcsim.gazebosim.org/src ${dist_version} main > /etc/apt/sources.list.d/src-latest.list"
          # 2. Install the SRCSim package
          $sh_c 'apt-get update; apt-get install -y -q srcsim'
          # 3. Change owner of ihmc_ros_java_adapter
          $sh_c "chown -R $USER:$USER /opt/ros/indigo/share/ihmc_ros_java_adapter"
          # 4. Add the ros group and add the user to it
          if ! getent group ros; then
	    $sh_c "groupadd ros"
          fi
          if ! groups ${USER} | grep -q ros; then 
            $sh_c "usermod -a -G ros $USER"
          fi
	  # 5. Limit file
          if [ ! -f /etc/security/limits.d/ros-rtprio.conf ]; then
            $sh_c "sudo bash -c 'echo \"@ros - rtprio 99\" > /etc/security/limits.d/ros-rtprio.conf'"
          fi
          # 6. Install a copy of gazebo_models
          if [ ! -d "${HOME}/.gazebo/models" ]; then
            mkdir -p "${HOME}/.gazebo/models"
			wget -qO- https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz | tar xvz --strip 1 -C ${HOME}/.gazebo/models
          fi
		  # 6. Install the valkyrie controller
          if [ ! -d "${HOME}/valkyrie" ]; then
            wget -qO- http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz | tar xvz -C ${HOME}
          fi
          )
          exit 0
        ;;
      esac
   ;;

   *)
     # intentionally mixed spaces and tabs here -- tabs are stripped by "<<-'EOF'", spaces are kept in the output
	cat >&2 <<- EOF

	  Your distribution ${lsb_dist}-${dist_version} is unsupported 
	  by the SRCSim project.

	  Current supported distributions:
	    - Ubuntu Trusty
	EOF
    exit 1
   ;;
  esac
}

# wrapped up in a function so that we have some protection against only getting
# half the file during "curl | sh"
do_install
