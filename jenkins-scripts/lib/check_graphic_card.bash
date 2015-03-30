GRAPHIC_CARD_FOUND=false
GRAPHIC_CARD_PKG=""

if [ -z ${GPU_SUPPORT_NEEDED} ]; then
    GPU_SUPPORT_NEEDED=false
fi

if ! ${GPU_SUPPORT_NEEDED}; then
    return
fi

# Hack to found the current display (if available) two steps:
# Check for /tmp/.X11-unix/ socket and check if the process is running
for i in `ls /tmp/.X11-unix/ | sed -e 's@^X@:@'`
do
  # grep can fail so let's disable the fail or error during its call
  set +e
  ps aux | grep bin/X.*$i | grep -v grep
  set -e
  if [ $? -eq 0 ] ; then
    export DISPLAY=$i
  fi
done

# Check for Nvidia stuff
if [ -n "$(lspci -v | grep nvidia | head -n 2 | grep "Kernel driver in use: nvidia")" ]; then
    export GRAPHIC_CARD_PKG=$(lspci -v | grep nvidia | head -n 2 | grep "Kernel modules:" | awk '{ print $3 }' | tr -d ','| sed -e s:_:-:g)
    if [ -z "${GRAPHIC_CARD_PKG}" ]; then
        # Trusty does not support the previous method. Fallback to use
	# installed package for GRAPHIC_CARD_PKG
	export GRAPHIC_CARD_PKG=$(dpkg -l | egrep "^ii[[:space:]]* nvidia-[0-9]{3} " | awk '{ print $2 }')
        if [ -z "${GRAPHIC_CARD_PKG}" ]; then
	  echo "Nvidia support found but not the module in use"
	  exit 1
        fi
    fi
    # Check for host installed version
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

# Check for intel
if [ -n "$(lspci -v | grep "Kernel driver in use: i[0-9][0-9][0-9]")" ]; then
    export GRAPHIC_CARD_PKG="xserver-xorg-video-intel"
    export GRAPHIC_CARD_NAME="Intel"
    export GRAPHIC_CARD_FOUND=true
    # Need to run properly DRI on intel
    export EXTRA_PACKAGES="${EXTRA_PACKAGES} libgl1-mesa-dri"
fi

# Be sure that we have GPU support
if $GPU_SUPPORT_NEEDED; then
    # Check for the lack of presence of DISPLAY var
    if [[ ${DISPLAY} == "" ]]; then
      echo "GPU support needed by the script but DISPLAY var is empty"
      # Try to restart lightdm. It should stop the script in the case of failure
      sudo service lightdm restart
    fi
    
    # Check if the GPU support was found when not 
    if ! $GRAPHIC_CARD_FOUND; then
      echo "GPU support needed by the script but no graphic card found."
      echo "The DISPLAY variable contains: ${DISPLAY}"
      exit 1
    fi
fi

# Get version of package 
export GRAPHIC_CARD_PKG_VERSION=$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print $3 }' | sed 's:-.*::')
echo "${GRAPHIC_CARD_NAME} found using package ${GRAPHIC_CARD_PKG} (${GRAPHIC_CARD_PKG_VERSION})"

