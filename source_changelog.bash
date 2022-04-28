#!/bin/bash
set -e

current_origin_remote=$(git config --get remote.origin.url)

# need tweaks if using ssh
if [[ "${current_origin_remote/git@}" != "${current_origin_remote}" ]]; then
  current_origin_remote=${current_origin_remote/\.git}
  current_origin_remote=${current_origin_remote/git@github.com:/https://github.com/}
fi

if [[ -z ${current_origin_remote} ]]; then
  echo "Script is unable to detect current remote.origin.url in this directory"
  exit 1
fi

if [[ $(git branch --show-current) == 'main' ]]; then
  echo "Error: main branch should not be used for releasing"
  exit 1
fi

if [[ -f "Changelog.md" ]]; then
  software_name=$(head -1 Changelog*)
  printf "%s (%s)\n\n" "${software_name}" "$(date +%Y-%m-%d)"
fi

current_tag=$(git symbolic-ref HEAD)
start_ref="HEAD"

# Find the previous release on the same branch, skipping prereleases if the
# current tag is a full release
previous_tag=""
while [[ -z $previous_tag || ( $previous_tag == *-* && $current_tag != *-* ) ]]; do
  previous_tag="$(git describe --tags "$start_ref"^ --abbrev=0)"
  start_ref="$previous_tag"
done

REPO=$(basename `git rev-parse --show-toplevel`)
REPO_FULL="${REPO/ign-/ignition-}"
VERSION=${previous_tag/${REPO_FULL}*_}
MAJOR=${VERSION%.*.*}
BRANCH=${REPO/sdformat/sdf}${MAJOR}

git log "${BRANCH}"..."${previous_tag}" --pretty=format:"%h|%s|%ae|%an" | \
  while IFS='|' read -r sha title author_email author || [[ -n ${sha} ]]; do
    pr_num="$(grep -o '#[[:digit:]]\+' <<<"$title")"
    pr_num="${pr_num:1}"
    pr_desc=${title/\ (#[[:digit:]]*)}
    printf "1. %s\n" "$pr_desc"
    printf "    * [Pull request #%s](%s/pull/%s)\n" "$pr_num" "$current_origin_remote" "$pr_num"
    if [[ "${author_email/@openrobotics.org}" == "${author_email}" ]] && \
       [[ "${author_email/@osrfoundation.org}" == "${author_email}" ]] && \
       [[ "${author_email/@users.noreply.github.com}" == "${author_email}" ]]; then
      printf "    * A contribution from: %s <%s>\n" "$author" "$author_email"
    fi
    echo
  done
