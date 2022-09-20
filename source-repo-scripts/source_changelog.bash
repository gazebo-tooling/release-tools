#!/bin/bash
# bash source_changelog.bash <PREV_VER>

PREV_VER=$1

git fetch --tags

REPO=$(basename $(git remote get-url origin))

# Find tag that ends with _$PREV_VER
PREV_TAG=$(git tag | grep "_${PREV_VER}$")

# Compare current branch to PREV_TAG
BRANCH=$(git rev-parse --abbrev-ref HEAD)

COMMITS=$(git log ${BRANCH}...${PREV_TAG}  --pretty=format:"%h")

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
