#!/bin/bash
# bash source_changelog.bash <PREV_VER>

PREV_TAG=$1

git fetch --tags

REPO=$(basename `git rev-parse --show-toplevel`)
REPO="${REPO/ign-/gz-}"
MAJOR=${PREV_TAG%.*.*}
BRANCH=${REPO/sdformat/sdf}${MAJOR}
TAG="${REPO}${MAJOR}"

# TODO(chapulina) Support Garden tags and branches, which will start with gz-
TAG="${TAG/gz-/ignition-}"
TAG="${TAG/sim/gazebo}"
BRANCH="${BRANCH/gz-/ign-}"
BRANCH="${BRANCH/sim/gazebo}"

COMMITS=$(git log ${BRANCH}...${TAG}_${PREV_TAG}  --pretty=format:"%h")

for COMMIT in $COMMITS
do
  TITLE_FULL=$(git log --format="%s" -n 1 $COMMIT)
  TITLE=${TITLE_FULL% (\#*)}
  PR=${TITLE_FULL#*\#}
  PR=${PR%)}

  echo "1. $TITLE"
  echo "    * [Pull request #$PR](https://github.com/gazebosim/$REPO/pull/$PR)"
  echo ""
done
