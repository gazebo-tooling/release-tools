GRAPHIC_CARD_FOUND=false
GRAPHIC_CARD_PKG=""

# Check for Nvidia stuff
NVIDIA_SUPPORT_RESULT=$(lspci -v | grep nvidia | grep "Kernel driver in use: nvidia")
if [ -n "${NVIDIA_SUPPORT_RESULT}" ]; then
    export GRAPHIC_CARD_PKG=$(lspci -v | grep nvidia | grep "Kernel modules:" | awk '{ print$3 }' | tr -d ','| sed -e s:_:-:)
    if [ -z ${GRAPHIC_CARD_PKG} ]; then
	echo "Nvidia support found but not the module in use"
	exit 1
    fi
    echo "Nvidia found using package ${GRAPHIC_CARD_PKG}"
    export GRAPHIC_CARD_FOUND=true
fi

# Check for ati stuff
ATI_SUPPORT_RESULT=$(lspci -v | grep "ATI" | grep "VGA")
if [ -n "${ATI_SUPPORT_RESULT}" ]; then
    # TODO search for correct version of fglrx
    export GRAPHIC_CARD_PKG=fglrx
    echo "ATI found using package ${GRAPHIC_CARD_PKG}"
    export GRAPHIC_CARD_FOUND=true
fi
