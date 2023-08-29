GRAPHIC_CARD_FOUND=false
GRAPHIC_CARD_PKG=""

export_display_variable()
{
    # Check for an active X11 display socket.
    for i in $(find /tmp/.X11-unix -type s)
    do
      # If a process is running with the open socket then
      # lsof will exit successfully.
      if sudo bash -c "lsof -Fp $i" ; then
        # Strip the path and leading X from the X11 socket
        # but check that the resulting string is numeric and
        # non-empty before exporting.
        DISPLAY=$(basename $i | sed -n -E 's/^X([0-9]+)/:\1/p')
        if [ -n "$DISPLAY" ]; then
          export DISPLAY
        fi
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

    # Implementation options (both methods can be coinstalled and runtime compatible)
    # 1. Native GPU Support:
    #   - Included with Docker-ce 19.03 or later
    #   - Package: nvidia-container-toolkit
    #   - docker param: --gpus all
    # 2. NVIDIA Container Runtime for Docker:
    #   - If the nvidia-docker2 package is installed
    #   - Package: nvidia-docker2
    #   - docker param: --runtime=nvidia
    #
    # Reference: https://docs.nvidia.com/deeplearning/frameworks/user-guide/index.html
    if dpkg -s nvidia-container-toolkit; then
      export NVIDIA_DOCKER_DRIVER='nvidia-container-toolkit'
    elif dpkg -s nvidia-docker2; then
      export NVIDIA_DOCKER_DRIVER='nvidia-docker2'
    elif dpkg -s nvidia-docker; then
      export NVIDIA_DOCKER_DRIVER='nvidia-docker'
    else
      export NVIDIA_DOCKER_DRIVER=''
    fi
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
      sudo systemctl restart lightdm
      # Wait for lightdm service to restart X11.
      sleep 5
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
  echo "${GRAPHIC_CARD_NAME} found using docker-nvidia support (${NVIDIA_DOCKER_DRIVER})"
fi
