#!/bin/bash
# Generates a list of changes since the last tagged version. 
#   bash source_changelog.bash
#
# Optionally, the previous version can be provided
# bash source_changelog.bash <PREV_VER>
#
#   E.g.
#   bash source_changelog.bash 3.0.0

git fetch --tags

PREV_VER=${1:-$(git describe --tags --abbrev=0 | sed 's/.*_//')}
echo "Changes since $PREV_VER"

ORIGIN_URL=$(git remote get-url origin)
REPO=$(basename ${ORIGIN_URL%.git})

# Find tag that ends with _$PREV_VER
PREV_TAG=$(git tag | grep "_${PREV_VER}$")

# Compare current branch to PREV_TAG
BRANCH=$(git rev-parse --abbrev-ref HEAD)

COMMITS=$(git log ${BRANCH}...${PREV_TAG} --no-merges --pretty=format:"%h")

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
