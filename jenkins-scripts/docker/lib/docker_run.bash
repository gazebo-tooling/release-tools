# TODO: run inside docker as a normal user and replace the sudo calls
# This are usually for debbuilders
export PACKAGE_DIR="${WORKSPACE}/pkgs"
export docker_cmd="docker"

# Do not delete packages from the scripts since some of them can be
# run twice from the same jenkins job and generate different pkgs
# If you want to do it, better do it from the DSL jenkins configuration
sudo mkdir -p ${PACKAGE_DIR}

# This are usually for continous integration jobs
sudo rm -fr ${WORKSPACE}/build
sudo mkdir -p ${WORKSPACE}/build

sudo docker build --tag ${DOCKER_TAG} .
stop_stopwatch CREATE_TESTING_ENVIROMENT

echo '# BEGIN SECTION: see build.sh script'
cat build.sh
echo '# END SECTION'

if $USE_GPU_DOCKER; then
  EXTRA_PARAMS_STR="--privileged \
                    -e DISPLAY=unix$DISPLAY \
                    -v /sys:/sys:ro         \
                    -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

  if [[ $GRAPHIC_CARD_NAME == "Nvidia" ]]; then
    export docker_cmd="nvidia-docker"
  fi
fi

if $ENABLE_CCACHE; then
  EXTRA_PARAMS_STR="$EXTRA_PARAMS_STR \
                    -v ${CCACHE_DIR}:${CCACHE_DIR}:rw"
fi

# DOCKER_FIX is for workaround https://github.com/docker/docker/issues/14203
sudo ${docker_cmd} run $EXTRA_PARAMS_STR  \
            -e DOCKER_FIX=''  \
            -e WORKSPACE=${WORKSPACE} \
            -e TERM=xterm-256color \
            -v ${WORKSPACE}:${WORKSPACE} \
            -v /dev/log:/dev/log:ro \
            -v /run/log:/run/log:ro \
            --tty \
            --rm \
            ${DOCKER_TAG} \
            /bin/bash build.sh

# Export results out of build directory, to WORKSPACE
for d in $(find ${WORKSPACE}/build -maxdepth 1 -name '*_results' -type d); do
    sudo mv ${d} ${WORKSPACE}/
    sudo chown -R jenkins ${WORKSPACE}/*_results
done

if [[ -z ${KEEP_WORKSPACE} ]]; then
    # Clean the whole build directory
    sudo rm -fr ${WORKSPACE}/build
    # Mimic old layout of exported test results
    mkdir ${WORKSPACE}/build
    # maxdepth is need to avoid problems finding build directories with
    # _results in the name
    for d in $(find ${WORKSPACE} -maxdepth 1 -name '*_results' -type d); do
       sudo mv ${d} ${WORKSPACE}/build/
    done
    
    [[ -d ${PACKAGE_DIR} ]] && sudo chown -R jenkins ${PACKAGE_DIR}
    sudo chown jenkins -R ${WORKSPACE}/build/
fi
