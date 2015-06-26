# TODO: run inside docker as a normal user and replace the sudo calls
# This are usually for debbuilders
sudo rm -fr ${WORKSPACE}/pkgs
sudo mkdir -p ${WORKSPACE}/pkgs
# This are usually for continous integration jobs
sudo rm -fr ${WORKSPACE}/build
sudo mkdir -p ${WORKSPACE}/build

sudo docker build -t ${DOCKER_TAG} .
stop_stopwatch CREATE_TESTING_ENVIROMENT

timing_mapping_str=""
if $ENABLE_TIMING; then
  timing_mapping_str="-v ${TIMING_DIR}:${TIMING_DIR}"
fi

sudo docker run  \
            --cidfile=${CIDFILE} \
            -v ${WORKSPACE}/pkgs:${WORKSPACE}/pkgs \
            -v ${WORKSPACE}/build:${WORKSPACE}/build \
	    $timing_mapping_str \
            -t ${DOCKER_TAG} \
            /bin/bash build.sh

CID=$(cat ${CIDFILE})

sudo docker stop ${CID} || true
sudo docker rm ${CID} || true

stop_stopwatch TOTAL_TIME
