#!/usr/bin/env bash

set -e

# Print a markdown summary of a release (not Changelog.md entries), with its
# changelog and contributors. The script is designed to publish release summaries
# from the internal Open Robotics team to the Community (usually in the public forum).
#
# cd <path_to_source_code>
# bash release_summary.bash 5.3.0 5.4.0

PREV=$1
NEW=$2

MAJOR=${PREV%.*.*}

if [[ ! -f CMakeLists.txt  ]]; then
  echo "No CMakeLists.txt detected. Are you in an source repository?"
  exit 1
fi

# Get lib name from CMakeLists project ()
LIB=$(sed -n 's/^project[[:space:]]*(\(ignition-\)\?\([a-Z|_]*\)[0-9]*.*)/\2/p' CMakeLists.txt)

if [ -z "${LIB}" ]; then
  echo "Parsing of CMakeLists.txt project tag failed"
  echo "Probably an internal bug"
  exit 1
fi

echo "---------------------------------"
# TODO(chapulina) Support gz tags
NAME_FOR_TAGS=ignition-${LIB}
NAME_FOR_TAGS="${NAME_FOR_TAGS/_/-}"
NAME_FOR_TAGS="${NAME_FOR_TAGS/ignition-sdformat/sdformat}"
echo "NAME_FOR_TAGS: $NAME_FOR_TAGS"

NAME_FOR_REPO=gz-${LIB}
NAME_FOR_REPO="${NAME_FOR_REPO/_/-}"
NAME_FOR_REPO="${NAME_FOR_REPO/gz-sdformat/sdformat}"
NAME_FOR_REPO="${NAME_FOR_REPO/gz-gazebo/gz-sim}"
echo "NAME_FOR_REPO: $NAME_FOR_REPO"

# TODO(chapulina) Support gz branches
NAME_FOR_BRANCH=ign-${LIB}
NAME_FOR_BRANCH="${NAME_FOR_BRANCH/_/-}"
NAME_FOR_BRANCH="${NAME_FOR_BRANCH/ign-sdformat/sdf}"
echo "NAME_FOR_BRANCH: $NAME_FOR_BRANCH"

NAME_FOR_TITLE="Gazebo ${LIB^}"
NAME_FOR_TITLE="${NAME_FOR_TITLE/Fuel_tools/Fuel Tools}"
NAME_FOR_TITLE="${NAME_FOR_TITLE/Gazebo Sdformat/SDFormat}"
NAME_FOR_TITLE="${NAME_FOR_TITLE/Gazebo Gazebo/Gazebo Sim}"
echo "NAME_FOR_TITLE: $NAME_FOR_TITLE"

echo "Previous version: ${PREV}"
echo "New version: ${NEW}"
echo "Major version: ${MAJOR}"
echo "---------------------------------"
echo ""

git fetch --all --tags

if ! git checkout "${NAME_FOR_BRANCH}${MAJOR}"; then
  echo "Branch ${NAME_FOR_BRANCH}${MAJOR} was not found in the repository"
  exit 1
fi

git pull origin ${NAME_FOR_BRANCH}${MAJOR}

echo ""
echo ""
echo ""
echo ""
echo "## $NAME_FOR_TITLE $NEW"
echo ""
echo "## Changelog"
echo ""
echo "[Full changelog](https://github.com/gazebosim/${NAME_FOR_REPO}/blob/${NAME_FOR_BRANCH}${MAJOR}/Changelog.md)"
echo ""
#awk '/${NEW}/{ f = 1; next } /${PREV}/{ f = 0 } f' Changelog.md
sed -n "/${NEW}/, /${PREV}/{ /${NEW}/! { /${PREV}/! p } }" Changelog.md
echo ""
echo "## Contributors"
echo ""
git log --pretty="%an" ${NAME_FOR_TAGS}${MAJOR}_${PREV}...${NAME_FOR_TAGS}${MAJOR}_${NEW} | sort | uniq | sed "s/.*/\*&\*/"
echo ""
echo "---"
