#!/bin/bash

# Print a summary of a release, with its changelog and contributors
#
# cd <path_to_source_code>
# bash release_summary.bash gui 5.3.0 5.4.0

LIB=$1
PREV=$2
NEW=$3

MAJOR=${PREV%.*.*}

echo "---------------------------------"
NAME_FOR_TAGS=ignition-${LIB}
NAME_FOR_TAGS="${NAME_FOR_TAGS/ignition-sdf/sdformat}"
echo "NAME_FOR_TAGS: $NAME_FOR_TAGS"

NAME_FOR_REPO=ign-${LIB}
NAME_FOR_REPO="${NAME_FOR_REPO/ign-sdf/sdformat}"
echo "NAME_FOR_REPO: $NAME_FOR_REPO"

NAME_FOR_BRANCH=ign-${LIB}
NAME_FOR_BRANCH="${NAME_FOR_BRANCH/ign-sdf/sdf}"
echo "NAME_FOR_BRANCH: $NAME_FOR_BRANCH"

NAME_FOR_TITLE="Ignition ${LIB^}"
NAME_FOR_TITLE="${NAME_FOR_TITLE/Ignition Sdf/SDFormat}"
echo "NAME_FOR_TITLE: $NAME_FOR_TITLE"

echo "Previous version: ${PREV}"
echo "New version: ${NEW}"
echo "Major version: ${MAJOR}"
echo "---------------------------------"
echo ""

git fetch --all --tags

git checkout ${NAME_FOR_BRANCH}${MAJOR}
git pull origin ${NAME_FOR_BRANCH}${MAJOR}

echo ""
echo ""
echo ""
echo ""
echo "## $NAME_FOR_TITLE $NEW"
echo ""
echo "## Changelog"
echo ""
echo "[Full changelog](https://github.com/ignitionrobotics/${NAME_FOR_REPO}/blob/${NAME_FOR_BRANCH}${MAJOR}/Changelog.md)"
echo ""
#awk '/${NEW}/{ f = 1; next } /${PREV}/{ f = 0 } f' Changelog.md
sed -n "/${NEW}/, /${PREV}/{ /${NEW}/! { /${PREV}/! p } }" Changelog.md
echo ""
echo "## Contributors"
echo ""
git log --pretty="%an" ${NAME_FOR_TAGS}${MAJOR}_${PREV}...${NAME_FOR_TAGS}${MAJOR}_${NEW} | sort | uniq | sed "s/.*/\*&\*/"
echo ""
echo "---"
