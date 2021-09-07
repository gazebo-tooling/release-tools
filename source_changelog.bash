#!/bin/bash

PREV_TAG=$1

git fetch --tags

REPO=$(basename `git rev-parse --show-toplevel`)
REPO_FULL="${REPO/ign-/ignition-}"
MAJOR=${PREV_TAG%.*.*}
BRANCH=${REPO/sdformat/sdf}${MAJOR}

COMMITS=$(git log ${BRANCH}...${REPO_FULL}${MAJOR}_${PREV_TAG}  --pretty=format:"%h")

for COMMIT in $COMMITS
do
  TITLE_FULL=$(git log --format="%s" -n 1 $COMMIT)
  TITLE=${TITLE_FULL%(\#*)}
  PR=${TITLE_FULL#*\#}
  PR=${PR%)}

  echo "1. $TITLE"
  echo "    * [Pull request #$PR](https://github.com/ignitionrobotics/$REPO/pull/$PR)"
  echo ""
done
