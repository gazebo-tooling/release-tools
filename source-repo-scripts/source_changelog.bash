#!/bin/bash
# Generates a list of changes since the last tagged version. 
#   bash source_changelog.bash
#
# Optionally, the previous version can be provided
# bash source_changelog.bash <PREV_VER>
#
#   E.g.
#   bash source_changelog.bash 3.0.0
#
# In a checkout that has a .changelog/ directory the changes are in its entry
# files, so the script runs create_changelog_entry.py instead of reading the
# git log.

# Print the given commits as Changelog.md items
print_commits() {
  for COMMIT in "$@"
  do
    TITLE_FULL=$(git log --format="%s" -n 1 $COMMIT)
    TITLE=${TITLE_FULL% (\#*)}
    PR=${TITLE_FULL#*\#}
    PR=${PR%)}

    echo "1. $TITLE"
    echo "    * [Pull request #$PR](https://github.com/gazebosim/$REPO/pull/$PR)"
    echo ""
  done
}

git fetch --tags

PREV_VER=${1:-$(git describe --tags --abbrev=0 | sed 's/.*_//')}

ORIGIN_URL=$(git remote get-url origin)
REPO=$(basename ${ORIGIN_URL%.git})

if [[ -d .changelog ]]; then
  # Git does not keep empty directories: without a tracked file, releasing the
  # last entries would remove .changelog/ and the next release would silently
  # go back to the git log.
  if ! git ls-files --error-unmatch .changelog/.gitkeep > /dev/null 2>&1; then
    echo "Warning: .changelog/.gitkeep is not tracked by git; add it or" \
      ".changelog/ will disappear along with its last entries." >&2
  fi

  # Commits merged before the first .changelog/ file have no entries. Several
  # tags can match the version (e.g. pre1), so take the nearest one.
  BASE_TAG=$(git describe --tags --abbrev=0 --match "*_${PREV_VER}" 2>/dev/null)
  if [[ -n ${BASE_TAG} ]] && ! git cat-file -e "${BASE_TAG}:.changelog" 2>/dev/null; then
    FIRST_ENTRY=$(git log --reverse --format=%H "${BASE_TAG}..HEAD" -- .changelog | head -n 1)
    if [[ -n ${FIRST_ENTRY} ]]; then
      COMMITS=$(git log "${BASE_TAG}..${FIRST_ENTRY}^" --no-merges --pretty=format:"%h")
    fi
    if [[ -n ${COMMITS} ]]; then
      {
        echo "Warning: these commits were merged after ${BASE_TAG} but before" \
          ".changelog/ existed, so no entry covers them. Add them to" \
          "Changelog.md by hand:"
        echo ""
        print_commits ${COMMITS}
      } >&2
    fi
  fi

  exec python3 "$(dirname "$0")/create_changelog_entry.py"
fi

echo "1. **Baseline:** this includes all changes from $PREV_VER and earlier."
echo ""

# Find tag that ends with _$PREV_VER
PREV_TAG=$(git tag | grep "_${PREV_VER}$")

# Compare current branch to PREV_TAG
BRANCH=$(git rev-parse --abbrev-ref HEAD)

COMMITS=$(git log ${BRANCH}...${PREV_TAG} --no-merges --pretty=format:"%h")

print_commits $COMMITS
