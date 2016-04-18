# TODO: run inside docker as a normal user and replace the sudo calls
# This are usually for debbuilders
sudo rm -fr ${WORKSPACE}/pkgs
sudo mkdir -p ${WORKSPACE}/pkgs
# This are usually for continous integration jobs
sudo rm -fr ${WORKSPACE}/build
sudo mkdir -p ${WORKSPACE}/build

sudo docker build --tag ${DOCKER_TAG} .
stop_stopwatch CREATE_TESTING_ENVIROMENT

if $USE_GPU_DOCKER; then
  EXTRA_PARAMS_STR="--privileged \
                    -e DISPLAY=unix$DISPLAY \
                    -v /sys:/sys:ro         \
                    -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
fi

if $ENABLE_CCACHE; then
  EXTRA_PARAMS_STR="$EXTRA_PARAMS_STR \
                    -v ${CCACHE_DIR}:${CCACHE_DIR}:rw"
fi

sudo docker run $EXTRA_PARAMS_STR  \
            --cidfile=${CIDFILE} \
            -v ${WORKSPACE}:${WORKSPACE} \
            --tty \
	    ${DOCKER_TAG} \
            /bin/bash build.sh

CID=$(cat ${CIDFILE})

sudo docker stop ${CID} || true
sudo docker rm ${CID} || true

# Export results out of build directory, to WORKSPACE
for d in $(find ${WORKSPACE}/build -name '*_results' -type d); do
    sudo mv ${d} ${WORKSPACE}/
    sudo chown -R jenkins ${WORKSPACE}/*_results
done

if [[ -z ${KEEP_WORKSPACE} ]]; then
    # Clean the whole build directory
    sudo rm -fr ${WORKSPACE}/build
    # Mimic old layout of exported test results
    mkdir ${WORKSPACE}/build
    for d in $(find ${WORKSPACE} -name '*_results' -type d); do
       sudo mv ${d} ${WORKSPACE}/build/
    done
    
    sudo chown jenkins -R ${WORKSPACE}/build/
fi
