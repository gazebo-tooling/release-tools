GRAPHIC_CARD_FOUND=false
GRAPHIC_CARD_PKG=""

if [ -z ${DISPLAY} ]; then
    return
fi

# Check for Nvidia stuff
if [ -n "$(lspci -v | grep nvidia | head -n 2 | grep "Kernel driver in use: nvidia")" ]; then
    export GRAPHIC_CARD_PKG=$(lspci -v | grep nvidia | head -n 2 | grep "Kernel modules:" | awk '{ print $3 }' | tr -d ','| sed -e s:_:-:)
    if [ -z "${GRAPHIC_CARD_PKG}" ]; then
	echo "Nvidia support found but not the module in use"
	exit 1
    fi
    # Check for host installed version
    export GRAPHIC_CARD_NAME="Nvidia"
    export GRAPHIC_CARD_FOUND=true
fi

# Check for ati stuff
ATI_SUPPORT_RESULT=
if [ -n "$(lspci -v | grep "ATI" | grep "VGA")" ]; then
    # TODO search for correct version of fglrx
    export GRAPHIC_CARD_PKG=fglrx
    export GRAPHIC_CARD_NAME="ATI"
    export GRAPHIC_CARD_FOUND=true
fi

# Get version of package 
export GRAPHIC_CARD_PKG_VERSION=$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print $3 }')
echo "${GRAPHIC_CARD_NAME} found using package ${GRAPHIC_CARD_PKG} (${GRAPHIC_CARD_PKG_VERSION})"

