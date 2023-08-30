#!/bin/bash -e

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <RELEASE_REPO_URL> <ROS_DISTRO> <GAZEBO_DISTRO>"
  exit 1
fi

RELEASE_REPO_URL=${1}
ROS_DISTRO=${2}
GAZEBO_DISTRO=${3}

echo " * Checking that ${RELEASE_REPO_URL} has a correct gz${GAZEBO_DISTRO} metadata for ROS ${ROS_DISTRO}"

TMP_DIR=$(mktemp -d) 
git clone -q "${RELEASE_REPO_URL}" "${TMP_DIR}"
pushd "${TMP_DIR}" 2> /dev/null > /dev/null || exit
DEBIAN_LATEST_TAG=$(git for-each-ref --sort=creatordate --format '%(refname)' refs/tags | \
          grep "debian/ros-${ROS_DISTRO}" | \
          tail -1 | sed 's:refs/tags/::')
git checkout -q "${DEBIAN_LATEST_TAG}"
if ! dpkg-parsechangelog -SSource 2> /dev/null | grep -q "gz${GAZEBO_DISTRO}"; then
  echo " !! Latest changelog entry does not have gz${GAZEBO_DISTRO} name"
  echo " !! in the repo tag '${DEBIAN_LATEST_TAG}' for repo '${RELEASE_REPO_URL}'"
  echo -n " !! Latest changelog is:"
  dpkg-parsechangelog -SSource 2> /dev/null || exit
  echo " !! Maybe the rename in the bloom templates was not done."
  exit 1
fi
popd >/dev/null || exit

echo "  + Changelog has correct gz${GAZEBO_DISTRO} metadata for ${DEBIAN_LATEST_TAG}"
