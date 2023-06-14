#!/bin/bash

set -e


if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <RELEASE_REPO_URL> <ROS_DISTRO>"
  exit 1
fi

RELEASE_REPO_URL=${1}
ROS_DISTRO=${2}

TMP_DIR=$(mktemp -d) 
git clone "${RELEASE_REPO_URL}" "${TMP_DIR}"
pushd "${TMP_DIR}" 2> /dev/null || exit
cd "${TMP_DIR}" || exit
DEBIAN_LATEST_TAG=$(git for-each-ref --sort=creatordate --format '%(refname)' refs/tags | \
          grep "debian/ros-${ROS_DISTRO}" | \
          tail -1 | sed 's:refs/tags/::')
git checkout "${DEBIAN_LATEST_TAG}"
dpkg-parsechangelog -SSource 
if ! dpkg-parsechangelog -SSource | grep -q 'gzgarden' > /dev/null; then
  echo "Latest changelog entry does not have gzgarden name"
  echo "in the repo tag ${DEBIAN_LATEST_TAG} for repo ${RELEASE_REPO_URL}"
  echo "Maybe the rename in the bloom templates was not done."
  exit 1
fi
popd 2>/dev/null || exit
