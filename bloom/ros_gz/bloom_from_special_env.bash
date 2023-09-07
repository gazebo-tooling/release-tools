#!/bin/bash -ex

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 <ROS_DISTRO> <TARGET_NEW_GAZEBO_COLLECTION> <URL_OSRF_ROSDEP_REPLACE>"
  echo "Example:"
  echo "  $0 humble garden https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/replace_fortress_with_garden/00-replace-gz-fortress-with-garden.list"
  exit 1
fi

ROS_DISTRO=${1}
GAZEBO_DISTRO=${2}
URL_OSRF_ROSDEP_REPLACE=${3}

# Might be used with testing proposes to change gbp-release repo
RELEASE_REPO_URL=${RELEASE_REPO_URL:-https://github.com/gazebo-release/ros_ign-release}

if ! command rocker --version &> /dev/null
then
  echo "Please install the rocker app https://github.com/osrf/rocker"
  exit 1
fi

RELEASE_REPO_TRACK=${ROS_DISTRO}_gz${GAZEBO_DISTRO}
RAW_RELEASE_REPO=${RELEASE_REPO_URL/github.com/raw.githubusercontent.com}

if ! curl --silent "${RAW_RELEASE_REPO}/master/tracks.yaml" | grep "${RELEASE_REPO_TRACK}" > /dev/null 2> /dev/null; then
  echo "Did not found ${RELEASE_REPO_TRACK} track in ${RAW_RELEASE_REPO}"
  exit 1
fi

# TODO: update URL for the release repo
BLOOM_CMD="/usr/bin/bloom-release --no-pull-request \
   --rosdistro ${ROS_DISTRO}\
   --track ${RELEASE_REPO_TRACK}  \
   --override-release-repository-url \
   ${RELEASE_REPO_URL} ros_gz"

TAG_NAME=${TAG_NAME:-ros_gz-${ROS_DISTRO}-${GAZEBO_DISTRO}-release}

echo " * Build the docker release environment"
docker build . -t "${TAG_NAME}" \
  --build-arg ROS_DISTRO="${ROS_DISTRO}" \
  --build-arg GAZEBO_DISTRO="${GAZEBO_DISTRO}" \
  --build-arg URL_OSRF_ROSDEP_REPLACE="${URL_OSRF_ROSDEP_REPLACE}"
echo " * Using rocker to enter in the release environment"
rocker --home --user "${TAG_NAME}" "${BLOOM_CMD}"
echo " * Exit the docker release environment"
echo " * Restoring the rosdep cache in the user"
rosdep update
echo " * Running safety check in generated metadata"
docker run "${TAG_NAME}" /tmp/_check_metadata.bash "${RELEASE_REPO_URL}" "${ROS_DISTRO}" "${GAZEBO_DISTRO}"
