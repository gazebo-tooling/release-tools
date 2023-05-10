#!/bin/bash

# TODO: update URL for the release repo
BLOOM_CMD="/usr/bin/bloom-release --no-pull-request \
                         --rosdistro humble \
                         --track humble_gzgarden \
                         --override-release-repository-url \
                         https://github.com/j-rivero/ros_ign-release ros_gz"
#
TAG_NAME=${TAG_NAME:-ros_gz_fortress_garden_release}

echo " * Build the docker release environment"
docker build . -t "${TAG_NAME}"
echo " * Using rocker to enter in the release environment"
rocker --home --user "${TAG_NAME}" "${BLOOM_CMD}"
echo " * Exit the docker release environment"
echo " * Restoring the rosdep cache in the user"
rosdep update
