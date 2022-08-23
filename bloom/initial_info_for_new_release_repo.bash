#!/bin/bash -e
#
# This script will populate an empty gazeboX_rosY_pkgs-release repository with
# information from current official -release repositories
#

if [[ ${#} -lt 2 ]]; then
    echo "Usage: ${0} <gazebo_major_version> [ros1|ros2]"
    exit -1
fi

GZ_MAJOR=${1}
ROS_VERSION=${2}

SOURCE_REPO="https://github.com/${ROS_VERSION}-gbp/gazebo_ros_pkgs-release"
DEST_REPO="https://github.com/osrf/gazebo${GZ_MAJOR}_${ROS_VERSION}_pkgs-release"
TEST_DIR=$(mktemp -d)

safety_check()
{
  RAW_URL="${DEST_REPO/github.com/raw.githubusercontent.com}/master/tracks.yaml"

  if wget -q "${RAW_URL}"; then
      echo "!! Looks like the ${DEST_REPO} exists!"
      echo "!! this tool is only for new repos"
      exit -1
  fi
}

clone_all_remote_branches()
{
    # source: https://coderwall.com/p/0ypmka
    for branch in `git branch -a | grep remotes | grep -v HEAD | grep -v master `; do
	   git branch --track ${branch#remotes/origin/} $branch
    done
}

echo "Summary: "
echo " - The source repo to use would be: ${SOURCE_REPO}"
echo " - The destination repo would be  : ${DEST_REPO}"

pushd ${TEST_DIR}
safety_check

git clone "${DEST_REPO}" dest_repo
git clone "${SOURCE_REPO}" source_repo
cp source_repo/* dest_repo/
cd dest_repo
git add *
git commit -m "Import metadata from gazebo_ros_pkgs repository"
git remote rename origin gazebo${GZ_MAJOR}
git remote add origin ${SOURCE_REPO}
git fetch origin
clone_all_remote_branches
git remote remove origin
git remote rename gazebo${GZ_MAJOR} origin
# 7. Push all the local branch and tags to the new repo
git push origin --all
git push origin --tags

echo " ** READY"
echo " remember to run: "
echo "   - rename-gazebo-ros-pkgs.bash ${GZ_MAJOR} <ros_distros>"
popd 2> /dev/null
