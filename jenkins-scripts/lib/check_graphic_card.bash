GRAPHIC_CARD_FOUND=false
GRAPHIC_CARD_PKG=""

export_display_variable()
{
    # Hack to found the current display (if available) two steps:
    # Check for /tmp/.X11-unix/ socket and check if the process is running
    for i in $(ls /tmp/.X11-unix/ | sed -e 's@^X@:@')
    do
      # grep can fail so let's disable the fail or error during its call
      set +e
      ps aux | grep bin/X.*$i | grep -v grep
      set -e
      if [ $? -eq 0 ] ; then
	export DISPLAY=$i
      fi
    done
}

if [ -z ${GPU_SUPPORT_NEEDED} ]; then
    GPU_SUPPORT_NEEDED=false
fi

if ! ${GPU_SUPPORT_NEEDED}; then
    return
fi

# First try to get the display variable
export_display_variable

# Check for intel
if [ -n "$(lspci -v | grep "Kernel driver in use: i[0-9][0-9][0-9]")" ]; then
    export GRAPHIC_CARD_PKG="xserver-xorg-video-intel"
    export GRAPHIC_CARD_NAME="Intel"
    export GRAPHIC_CARD_FOUND=true
    # Need to run properly DRI on intel
    export EXTRA_PACKAGES="${EXTRA_PACKAGES} libgl1-mesa-dri"
fi

# If there are two cards in use (intel + other), let nvidia override and be the one selected
# Check for Nvidia stuff. Using nvidia-docker no installation needed
if [ -n "$(lspci -v | grep nvidia | head -n 2 | grep "Kernel driver in use: nvidia")" ]; then
    export GRAPHIC_CARD_NAME="Nvidia"
    export GRAPHIC_CARD_FOUND=true
fi

# Check for ati stuff
if [ -n "$(lspci -v | grep "ATI" | grep "VGA")" ]; then
    # TODO search for correct version of fglrx
    export GRAPHIC_CARD_PKG=fglrx
    export GRAPHIC_CARD_NAME="ATI"
    export GRAPHIC_CARD_FOUND=true
fi

# Be sure that we have GPU support
if $GPU_SUPPORT_NEEDED; then
    # Check for the lack of presence of DISPLAY var
    if [[ ${DISPLAY} == "" ]]; then
      echo "GPU support needed by the script but DISPLAY var is empty"
      # Try to restart lightdm. It should stop the script in the case of failure
      sudo service lightdm restart
      # Second try to get display variable
      export_display_variable
      if [[ ${DISPLAY} == "" ]]; then
	  echo "Imposible to get DISPLAY variable. Check your system"
	  exit 1
      fi
    fi
    
    # Check if the GPU support was found when not 
    if ! $GRAPHIC_CARD_FOUND; then
      echo "GPU support needed by the script but no graphic card found."
      echo "The DISPLAY variable contains: ${DISPLAY}"
      exit 1
    fi
fi


# Get version of package if needed
if [[ $GRAPHIC_CARD_NAME != "Nvidia" ]];then
  export GRAPHIC_CARD_PKG_VERSION=$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print $3 }' | sed 's:-.*::')
  echo "${GRAPHIC_CARD_NAME} found using package ${GRAPHIC_CARD_PKG} (${GRAPHIC_CARD_PKG_VERSION})"
else
  echo "${GRAPHIC_CARD_NAME} found using docker-nvidia wrapper"
fi
