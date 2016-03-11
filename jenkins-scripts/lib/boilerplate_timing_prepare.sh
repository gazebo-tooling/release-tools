# TIMING ROUTINES
if [ -z ${ENABLE_TIMING} ]; then
  ENABLE_TIMING=false
fi

# Need to copy the library into WORKSPACE/script to be used 
# inside containers/chroot
TIMING_DIR=${WORKSPACE}/timing
mkdir -p ${TIMING_DIR}
rm -f ${TIMING_DIR}/*

# Workaround for docker/chroot scripts
if [[ -f ${SCRIPT_DIR}/lib/_time_lib.sh ]]; then
  cp ${SCRIPT_DIR}/lib/_time_lib.sh ${TIMING_DIR}
else
  cp ${SCRIPT_DIR}/../lib/_time_lib.sh ${TIMING_DIR}
fi

if $ENABLE_TIMING; then
  touch $TIMING_DIR/_enable_timing
fi

# Always source timing. If not enable it will generate stubs for functions
. ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}
