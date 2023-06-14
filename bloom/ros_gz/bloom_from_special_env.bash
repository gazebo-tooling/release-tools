#!/bin/bash

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <ros_distro>"
  exit 1
fi

ROS_DISTRO=${1}
# Might be used with testing proposes to change gbp-release repo
GBP_ORG=${GBP_ORG:-gazebo-release}

if ! command rocker --version &> /dev/null
then
  echo "Please install the rocker app https://github.com/osrf/rocker"
  exit 1
fi

# TODO: update URL for the release repo
BLOOM_CMD="/usr/bin/bloom-release --no-pull-request \
   --rosdistro ${ROS_DISTRO}\
   --track ${ROS_DISTRO}_gzgarden \
   --override-release-repository-url \
   https://github.com/${GBP_ORG}/ros_ign-release ros_gz"

TAG_NAME=${TAG_NAME:-ros_gz-${ROS_DISTRO}-fortress-garden-release}

echo " * Build the docker release environment"
docker build . -t "${TAG_NAME}" --build-arg ROS_DISTRO="${ROS_DISTRO}"
echo " * Using rocker to enter in the release environment"
rocker --home --user "${TAG_NAME}" "${BLOOM_CMD}"
echo " * Exit the docker release environment"
echo " * Restoring the rosdep cache in the user"
rosdep update
